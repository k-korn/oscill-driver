// End-to-end smoke test: loads the plugin through libscopehal's normal plugin path ($HOME/.scopehal/plugins),
// connects to a serial port (real probe or tools/oscill_sim.py) and captures a few waveforms.
//
//   smoke_test /dev/ttyUSB0:9600 [count]

#include "scopehal.h"
#include "EdgeTrigger.h"

#include <atomic>
#include <cmath>
#include <thread>
#include <unistd.h>

using namespace std;

static bool Capture(shared_ptr<Oscilloscope> scope, int count, double timeout)
{
	double deadline = GetTime() + timeout;
	int got = 0;
	while(got < count && GetTime() < deadline)
	{
		if(!scope->IsTriggerArmed())
			break;
		if(scope->PollTrigger() == Oscilloscope::TRIGGER_MODE_TRIGGERED && scope->AcquireData())
		{
			scope->PopPendingWaveform();
			auto wfm = dynamic_cast<UniformAnalogWaveform*>(scope->GetOscilloscopeChannel(0)->GetData(0));
			if(!wfm)
				return false;
			wfm->PrepareForCpuAccess();
			float vmin = 1e9, vmax = -1e9;
			for(size_t i=0; i<wfm->size(); i++)
			{
				vmin = min(vmin, wfm->m_samples[i]);
				vmax = max(vmax, wfm->m_samples[i]);
			}
			LogNotice("waveform %d: %zu samples, %.3f us/sample, min %.3f V, max %.3f V\n",
				got, wfm->size(), wfm->m_timescale / 1e9, vmin, vmax);
			got++;
		}
	}
	return got == count;
}

/**
	@brief Trigger alignment check, for use with a periodic signal on the tip (e.g. 50 Hz mains hum from a finger).

	In Normal mode every waveform must cross the trigger level, in the selected direction, at the trigger offset.
 */
static bool CheckTriggerAlignment(shared_ptr<Oscilloscope> scope, EdgeTrigger::EdgeType type, const char* name)
{
	auto et = dynamic_cast<EdgeTrigger*>(scope->GetTrigger());
	et->SetType(type);
	et->SetLevel(0);
	scope->PushTrigger();
	scope->Start();

	int good = 0, total = 0;
	double deadline = GetTime() + 20;
	while(total < 5 && GetTime() < deadline)
	{
		if(scope->PollTrigger() != Oscilloscope::TRIGGER_MODE_TRIGGERED || !scope->AcquireData())
			continue;
		scope->PopPendingWaveform();
		auto w = dynamic_cast<UniformAnalogWaveform*>(scope->GetOscilloscopeChannel(0)->GetData(0));
		w->PrepareForCpuAccess();

		//Index of the sample drawn at the trigger offset
		double idx = (double)(scope->GetTriggerOffset() - w->m_triggerPhase) / w->m_timescale;
		//The crossing lies between the two samples straddling idx (works for single-sample pulses too)
		long i0 = (long)floor(idx);
		if(idx == floor(idx))
			i0--;			//crossing exactly on a sample: compare it with the one before
		if(i0 < 0 || i0 + 1 >= (long)w->size())
		{
			LogNotice("  %s #%d: trigger point at sample %.1f is outside the record\n", name, total, idx);
			total++;
			continue;
		}
		float before = w->m_samples[i0];
		float after = w->m_samples[i0 + 1];
		bool ok = (type == EdgeTrigger::EDGE_RISING) ? (before < 0 && after >= 0) : (before > 0 && after <= 0);
		LogNotice("  %s #%d: crossing at sample %.2f, %.1f mV -> %.1f mV  %s\n",
			name, total, idx, before * 1e3, after * 1e3, ok ? "OK" : "WRONG");
		good += ok;
		total++;
	}
	scope->Stop();
	LogNotice("  %s: %d / %d aligned\n", name, good, total);
	return (total >= 3) && (good == total);
}

int main(int argc, char** argv)
{
	g_log_sinks.emplace(g_log_sinks.begin(), new ColoredSTDLogSink(
		getenv("OSCILL_TRACE") ? Severity::TRACE : Severity::DEBUG));

	if(argc < 2)
	{
		LogError("usage: %s /dev/ttyUSB0:921600 [count] [trigger|filters|stress]\n", argv[0]);
		return 2;
	}
	int count = (argc > 2) ? atoi(argv[2]) : 3;

	if(!VulkanInit(true))	//headless
		_exit(1);	//not return: see the _exit() at the end
	TransportStaticInit();
	DriverStaticInit();
	InitializePlugins();

	vector<string> drivers;
	Oscilloscope::EnumDrivers(drivers);
	if(find(drivers.begin(), drivers.end(), "oscill") == drivers.end())
	{
		LogError("driver \"oscill\" not registered - is liboscill.so in ~/.scopehal/plugins?\n");
		_exit(1);	//not return: see the _exit() at the end
	}

	auto transport = SCPITransport::CreateTransport("uart", argv[1]);
	if(!transport || !transport->IsConnected())
	{
		LogError("could not open %s\n", argv[1]);
		_exit(1);	//not return: see the _exit() at the end
	}
	auto scope = Oscilloscope::CreateOscilloscope("oscill", transport);
	if(!scope)
		_exit(1);	//not return: see the _exit() at the end
	scope->m_nickname = "oscill";
	LogNotice("Connected: vendor \"%s\", model \"%s\", serial \"%s\", IDPing \"%s\"\n",
		scope->GetVendor().c_str(), scope->GetName().c_str(), scope->GetSerial().c_str(), scope->IDPing().c_str());

	bool ok = true;

	if( (argc > 3) && (string(argv[3]) == "stress") )
	{
		//Mimics ngscopeclient: one thread acquires while another changes settings, replacing the trigger object
		//(and deleting the old one) without the driver's lock, like Oscilloscope::SetTrigger() from the GUI.
		LogNotice("--- stress: acquisition thread vs settings thread for 30 s ---\n");
		atomic<bool> done(false);
		atomic<int> waveforms(0), changes(0);
		scope->StartAutoTrigger();
		thread acq([&]()
		{
			while(!done)
			{
				if(scope->PollTrigger() == Oscilloscope::TRIGGER_MODE_TRIGGERED && scope->AcquireData())
				{
					scope->PopPendingWaveform();
					waveforms++;
				}
			}
		});
		static const float ranges[] = { 0.8, 1.6, 4, 8 };
		static const uint64_t rates[] = { 6400, 20000, 100000, 200000 };
		double end = GetTime() + 30;
		for(int k = 0; GetTime() < end; k++)
		{
			switch(k % 4)
			{
				case 0:
				{
					auto et = new EdgeTrigger(scope.get());
					et->SetInput(0, StreamDescriptor(scope->GetOscilloscopeChannel(0), 0), true);
					et->SetLevel(0.05 * (k % 7) - 0.15);
					et->SetType((k & 4) ? EdgeTrigger::EDGE_FALLING : EdgeTrigger::EDGE_RISING);
					scope->SetTrigger(et);		//deletes the previous trigger object
					break;
				}
				case 1:
				{
					auto et = dynamic_cast<EdgeTrigger*>(scope->GetTrigger());
					et->SetLevel(0.02 * (k % 11) - 0.1);
					scope->PushTrigger();
					break;
				}
				case 2:
					scope->SetChannelVoltageRange(0, 0, ranges[(k / 4) % 4]);
					break;
				case 3:
					scope->SetSampleRate(rates[(k / 4) % 4]);
					break;
			}
			changes++;
			this_thread::sleep_for(chrono::milliseconds(20));
		}
		done = true;
		acq.join();
		scope->Stop();
		LogNotice("STRESS: %d waveforms, %d settings changes\n", waveforms.load(), changes.load());
		bool ok = waveforms > 20;
		LogNotice("%s\n", ok ? "STRESS TEST PASSED" : "STRESS TEST FAILED");
		scope = nullptr;
		ScopehalStaticCleanup();
		fflush(stdout);
		fflush(stderr);
		_exit(ok ? 0 : 1);
	}

	if( (argc > 3) && (string(argv[3]) == "filters") )
	{
		//Needs a spiky signal on the tip. The high-pass ("Filt H") turns sharp edges into decaying tails and removes
		//slow content; the 3 MHz low-pass ("Filt L") has no visible effect at 200 kS/s.
		LogNotice("--- input filters via ADC mode (spiky signal on the tip) ---\n");
		scope->SetChannelCoupling(0, OscilloscopeChannel::COUPLE_DC_1M);
		scope->SetChannelVoltageRange(0, 0, 1.6);	//0.2 V/div
		scope->SetSampleRate(200000);
		auto names = scope->GetADCModeNames(0);
		for(size_t mode = 0; mode < names.size(); mode++)
		{
			scope->SetADCMode(0, mode);
			scope->StartAutoTrigger();
			double sumPeak = 0, sumWidth = 0;
			int spikes = 0, records = 0;
			double deadline = GetTime() + 20;
			while(spikes < 5 && GetTime() < deadline)
			{
				if(scope->PollTrigger() != Oscilloscope::TRIGGER_MODE_TRIGGERED || !scope->AcquireData())
					continue;
				scope->PopPendingWaveform();
				records++;
				auto w = dynamic_cast<UniformAnalogWaveform*>(scope->GetOscilloscopeChannel(0)->GetData(0));
				w->PrepareForCpuAccess();
				size_t ipk = 0;
				for(size_t k=0; k<w->size(); k++)
					if(fabs(w->m_samples[k]) > fabs(w->m_samples[ipk]))
						ipk = k;
				float pk = w->m_samples[ipk];
				if(fabs(pk) < 0.075)
					continue;		//no spike in this record
				size_t a = ipk, b = ipk;
				while(a > 0 && w->m_samples[a-1] * pk > 0 && fabs(w->m_samples[a-1]) >= fabs(pk) / 2) a--;
				while(b + 1 < w->size() && w->m_samples[b+1] * pk > 0 && fabs(w->m_samples[b+1]) >= fabs(pk) / 2) b++;
				sumPeak += fabs(pk);
				sumWidth += b - a + 1;
				spikes++;
			}
			scope->Stop();
			LogNotice("  ADC mode %zu \"%s\" (BW limit reads %u MHz): %d spikes in %d records, |peak| %.0f mV, width %.1f samples\n",
				mode, names[mode].c_str(), scope->GetChannelBandwidthLimit(0), spikes, records,
				spikes ? sumPeak / spikes * 1e3 : 0, spikes ? sumWidth / spikes : 0);
		}
		scope->SetADCMode(0, 0);
		scope = nullptr;
		ScopehalStaticCleanup();
		fflush(stdout);
		fflush(stderr);
		_exit(0);
	}

	if( (argc > 3) && (string(argv[3]) == "trigger") )
	{
		LogNotice("--- software trigger alignment (hold the probe tip) ---\n");
		scope->SetChannelVoltageRange(0, 0, 0.8);		//0.1 V/div
		scope->SetSampleRate(6400);						//~40 ms per record = 2 mains cycles
		scope->SetTriggerOffset(scope->GetSampleDepth() / 2 * (FS_PER_SECOND / scope->GetSampleRate()));
		bool ok = CheckTriggerAlignment(scope, EdgeTrigger::EDGE_RISING, "rising");
		ok &= CheckTriggerAlignment(scope, EdgeTrigger::EDGE_FALLING, "falling");
		LogNotice("%s\n", ok ? "TRIGGER TEST PASSED" : "TRIGGER TEST FAILED");
		scope = nullptr;
		ScopehalStaticCleanup();
		fflush(stdout);
		fflush(stderr);
		_exit(ok ? 0 : 1);
	}

	LogNotice("--- auto trigger ---\n");
	scope->StartAutoTrigger();
	ok &= Capture(scope, count, 20);

	LogNotice("--- change settings while armed (exercises cancel) ---\n");
	scope->SetChannelVoltageRange(0, 0, 16);	//2 V/div
	scope->SetSampleRate(100000);
	auto et = dynamic_cast<EdgeTrigger*>(scope->GetTrigger());
	et->SetLevel(0);		//0 V: with the software trigger, Normal/Single need a signal crossing it
	et->SetType(EdgeTrigger::EDGE_FALLING);
	scope->PushTrigger();
	LogNotice("range %.1f V, rate %llu Hz\n", scope->GetChannelVoltageRange(0, 0), (unsigned long long)scope->GetSampleRate());

	LogNotice("--- normal trigger ---\n");
	scope->Start();
	ok &= Capture(scope, count, 20);

	LogNotice("--- single + stop ---\n");
	scope->StartSingleTrigger();
	ok &= Capture(scope, 1, 10);
	ok &= !scope->IsTriggerArmed();

	LogNotice("--- force trigger ---\n");
	scope->ForceTrigger();
	ok &= Capture(scope, 1, 10);

	scope->Stop();
	scope = nullptr;	//also closes the transport

	LogNotice("%s\n", ok ? "SMOKE TEST PASSED" : "SMOKE TEST FAILED");
	ScopehalStaticCleanup();

	//With an installed libscopehal the test carries its own copy of the logging code, whose globals interpose
	//libscopehal's; both would destroy them at exit (double free). Skip static destructors.
	fflush(stdout);
	fflush(stderr);
	_exit(ok ? 0 : 1);
}
