// End-to-end smoke test: loads the plugin through libscopehal's normal plugin path ($HOME/.scopehal/plugins),
// connects to a serial port (real probe or tools/oscill_sim.py) and captures a few waveforms.
//
//   smoke_test /dev/ttyUSB0:9600 [count]

#include "scopehal.h"
#include "EdgeTrigger.h"

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

int main(int argc, char** argv)
{
	g_log_sinks.emplace(g_log_sinks.begin(), new ColoredSTDLogSink(
		getenv("OSCILL_TRACE") ? Severity::TRACE : Severity::DEBUG));

	if(argc < 2)
	{
		LogError("usage: %s /dev/ttyUSB0:9600 [count]\n", argv[0]);
		return 2;
	}
	int count = (argc > 2) ? atoi(argv[2]) : 3;

	if(!VulkanInit(true))	//headless
		return 1;
	TransportStaticInit();
	DriverStaticInit();
	InitializePlugins();

	vector<string> drivers;
	Oscilloscope::EnumDrivers(drivers);
	if(find(drivers.begin(), drivers.end(), "oscill") == drivers.end())
	{
		LogError("driver \"oscill\" not registered - is liboscill.so in ~/.scopehal/plugins?\n");
		return 1;
	}

	auto transport = SCPITransport::CreateTransport("uart", argv[1]);
	if(!transport || !transport->IsConnected())
	{
		LogError("could not open %s\n", argv[1]);
		return 1;
	}
	auto scope = Oscilloscope::CreateOscilloscope("oscill", transport);
	if(!scope)
		return 1;
	scope->m_nickname = "oscill";
	LogNotice("Connected: vendor \"%s\", model \"%s\", serial \"%s\", IDPing \"%s\"\n",
		scope->GetVendor().c_str(), scope->GetName().c_str(), scope->GetSerial().c_str(), scope->IDPing().c_str());

	bool ok = true;

	LogNotice("--- auto trigger ---\n");
	scope->StartAutoTrigger();
	ok &= Capture(scope, count, 20);

	LogNotice("--- change settings while armed (exercises cancel) ---\n");
	scope->SetChannelVoltageRange(0, 0, 16);	//2 V/div
	scope->SetSampleRate(100000);
	auto et = dynamic_cast<EdgeTrigger*>(scope->GetTrigger());
	et->SetLevel(0.5);
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
