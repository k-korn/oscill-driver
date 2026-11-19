/***********************************************************************************************************************
*                                                                                                                      *
* Oscill DSO1 plugin for ngscopeclient / libscopehal                                                                   *
*                                                                                                                      *
***********************************************************************************************************************/

#include "scopehal.h"
#include "EdgeTrigger.h"
#include "OscillOscilloscope.h"

#include <cmath>
#include <algorithm>
#include <regex>
#include <unistd.h>

using namespace std;
using namespace oscill;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Hardware mappings
//
// Register widths (1 / 2 / 4 bytes) and the V1 / TS / MC formulas were recovered from oscill.exe
// and verified on hardware where noted (see docs/Protocol.md). Items marked GUESS have not been confirmed.
//   1 byte: O1 M1 T1 S1 RT AP AR RS   2 bytes: MC V1 QS TC   4 bytes: TS TD TA TW

//Run modes written to the RT register. The driver only uses RUN_FREE (0, verified); the other values follow the
//original guess and are unconfirmed (see docs/Protocol.md, "Trigger").
enum RunMode : uint8_t
{
	RUN_FREE	= 0,
	RUN_AUTO	= 1,
	RUN_NORMAL	= 2,
	RUN_WAIT	= 3
};

//O1 = input option bits (oscill.exe 0x4b4f50). M1 is the acquisition mode (sample/avg/hires/peak), not coupling,
//so the driver leaves M1 alone.
//Confirmed with a 1.36 V battery at 0.5 V/div: O1=0 reads 214 (DC), O1=2 reads 128 (DC blocked, AC signals still pass:
//AC coupling), O1=1 and 3 read a noiseless 128 (input grounded).
static const uint8_t O1_GROUND		= 0x01;
static const uint8_t O1_AC			= 0x02;
static const uint8_t O1_COUPLING	= O1_GROUND | O1_AC;

//Input filters: the vendor app's "Filt L" / "Filt H" buttons = low-pass / high-pass.
//Bit 3 is a HIGH-pass (~3 kHz): a 500 mV 50 Hz signal dropped to ~5 mV (first-order 3 kHz: 1.7 %), while fast spike
//edges passed and decayed into tails. Bit 2 is presumably the ~3 MHz LOW-pass: no visible effect at <= 200 kS/s.
static const uint8_t O1_LOWPASS_3MHZ	= 0x04;		//"Filt L"
static const uint8_t O1_HIGHPASS_3KHZ	= 0x08;		//"Filt H"
static const uint8_t O1_FILTERS			= O1_LOWPASS_3MHZ | O1_HIGHPASS_3KHZ;

//Shown by ngscopeclient as ADC modes. Only the low-pass maps to the bandwidth-limit API.
enum FilterMode { FILTER_NONE = 0, FILTER_LOWPASS = 1, FILTER_HIGHPASS = 2, FILTER_BANDPASS = 3 };
static const uint8_t g_filterBits[] = { 0, O1_LOWPASS_3MHZ, O1_HIGHPASS_3KHZ, O1_LOWPASS_3MHZ | O1_HIGHPASS_3KHZ };

//T1 = trigger configuration (oscill.exe 0x4b7770)
static const uint8_t T1_HYST_MASK	= 0x0f;		//bits 0-1 fall hysteresis, bits 2-3 rise hysteresis
static const uint8_t T1_FALLING		= 0x10;
static const uint8_t T1_RISING		= 0x20;
static const uint8_t T1_EDGE_MASK	= T1_FALLING | T1_RISING;

//Timer registers (TD trigger delay, TA / TW auto / wait timeouts) count units of 12000 MCU clocks
static const double TIMER_TICK_CLOCKS = 12000;
static const double AUTO_TIMEOUT_S = 0.1;		//Auto mode: free-run after this long without a trigger

//Software trigger (the hardware trigger does not gate captures on fw 1.26; see docs/Protocol.md)
static const size_t JUNK_SAMPLES = 2;			//first samples of every record are garbage (~250, ~66)
static const double SW_TRIGGER_HYST_CODES = 3;	//noise is about +-2..3 codes

//Vertical scale. V1 = volts/div in millivolts (oscill.exe: V1 = range * 1000 / 8, 8 divisions full screen).
//The 1-2-5 steps match the vendor UI; the exact span the hardware accepts is not known.
//Same steps as the vendor app (20 mV/div .. 10 V/div)
static const double g_voltsPerDiv[] = { 0.02, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0 };
static const size_t g_numVoltRanges = sizeof(g_voltsPerDiv) / sizeof(g_voltsPerDiv[0]);
static const size_t g_defaultVoltRange = 5;		//1 V/div

//The 8-bit ADC spans the 8 screen divisions, with code 128 at 0 V. Confirmed with a 1.361 V battery: 0.5 V/div gave
//215.0 / 128.2 / 42.2 (+, shorted, reversed), 1 V/div gave 85.0 and 5 V/div 119.4 (reversed); within 1% everywhere.
static const double g_adcDivisions = 8;
static const int g_adcZeroCode = 128;

//Timebase (oscill.exe): TS = sample period in MCU clock cycles, 8.8 fixed point, limited to [TMl, TMh].
//MCU clock period = MC * 10 ps (MCl / MCh give the legal MC range).
static const int64_t MC_UNIT_FS			= 10000;		//10 ps
static const uint32_t DEFAULT_MC		= 1250;			//12.5 ns / 80 MHz, used if MC can't be read
static const uint32_t DEFAULT_TS_MIN	= 256;			//one sample per clock (faster needs RIS, not supported)
static const uint32_t DEFAULT_TS_MAX	= 0x7fffffff;	//4-byte register; ~1 sample per 2 s at 80 MHz
static const uint64_t DEFAULT_SAMPLE_RATE = 1000000;


//Timeouts
static const int CONNECT_TIMEOUT_MS		= 1000;
static const int POLL_TIMEOUT_MS		= 50;		//how long one PollTrigger() call may block
static const double FREERUN_WATCHDOG_S	= 5.0;		//free-run / auto replies should never take this long

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

OscillOscilloscope::OscillOscilloscope(SCPITransport* transport)
	: SCPIDevice(transport, false)
	, SCPIInstrument(transport, false)
	, m_connected(false)
	, m_requestOutstanding(false)
	, m_frameReady(false)
	, m_autoMode(false)
	, m_forceNext(false)
	, m_requestTime(0)
	, m_currentRunMode(-1)
	, m_channelEnabled(true)
	, m_coupling(OscilloscopeChannel::COUPLE_DC_1M)
	, m_attenuation(1)
	, m_offset(0)
	, m_vrangeIndex(g_defaultVoltRange)
	, m_mc(DEFAULT_MC)
	, m_tsMin(DEFAULT_TS_MIN)
	, m_tsMax(DEFAULT_TS_MAX)
	, m_ts(0)
	, m_sampleRate(DEFAULT_SAMPLE_RATE)
	, m_tcMax(0xffff)
	, m_o1(0)
	, m_t1(0)
	, m_triggerOffset(0)
	, m_swTriggerPhase(0)
	, m_swTrigLevel(0)
	, m_swTrigEdges(EDGE_RISING_BIT)
	, m_lastDepth(NOMINAL_SAMPLE_COUNT)
	, m_regWriteFailures(0)
{
	m_vendor = "OKB ROSA";
	m_model = "Oscill DSO1";
	m_serial = "";
	m_fwVersion = "";

	m_ts = RateToTS(m_sampleRate);

	m_channels.push_back(new OscilloscopeChannel(
		this,
		"CH1",
		"#ffff00",
		Unit(Unit::UNIT_FS),
		Unit(Unit::UNIT_VOLTS),
		Stream::STREAM_TYPE_ANALOG,
		0));

	//Trigger point in the middle of the record by default
	m_triggerOffset = GetSamplePeriodFs() * (int64_t)(NOMINAL_SAMPLE_COUNT / 2);

	lock_guard<recursive_mutex> lock(m_mutex);
	if(!m_port.Attach(transport))
		return;

	m_connected = Connect();
	if(!m_connected)
	{
		LogError("Oscill: probe did not answer the OBEX CONNECT handshake\n");
		return;
	}

	//Trigger point in the middle of the record actually delivered (QS minus the junk samples)
	if(m_lastDepth > JUNK_SAMPLES)
		m_triggerOffset = GetSamplePeriodFs() * (int64_t)((m_lastDepth - JUNK_SAMPLES) / 2);

	PullTrigger();
	PushAll();
}

OscillOscilloscope::~OscillOscilloscope()
{
	lock_guard<recursive_mutex> lock(m_mutex);
	if(m_connected)
	{
		CancelPendingAcquisition();

		//Leave the probe at its power-on rate, as the vendor software does
		//(its ACK comes back at 9600 and would be garbage at our rate: just let the command go out)
		if(m_port.GetBaudRate() != 9600)
		{
			SendFrame(BuildSetSpeed(9600));
			usleep(50 * 1000);
		}
	}
}

string OscillOscilloscope::GetDriverNameInternal()
{
	return "oscill";
}

unsigned int OscillOscilloscope::GetInstrumentTypes() const
{
	return Instrument::INST_OSCILLOSCOPE;
}

uint32_t OscillOscilloscope::GetInstrumentTypesForChannel(size_t /*i*/) const
{
	return Instrument::INST_OSCILLOSCOPE;
}

string OscillOscilloscope::IDPing()
{
	//GET 'F' is rejected by the oscilloscope firmware (it appears to be a bootloader command), so ping by reading
	//the identity registers fresh
	lock_guard<recursive_mutex> lock(m_mutex);
	string id;
	RegValue v;
	for(auto name : { "VNM", "VSN", "VSW" })
	{
		if(ReadRegister(name, v))
			id += (id.empty() ? "" : " ") + v.AsText();
	}
	return id;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Low level protocol

/**
	@brief OBEX CONNECT handshake followed by identification queries
 */
bool OscillOscilloscope::Connect()
{
	Bytes reply;
	bool ok = false;

	unsigned int wantedBaud = m_port.GetBaudRate();

	//Try the configured baud rate first (the probe may still be there from an earlier session),
	//then the 9600 baud power-on default
	for(int attempt = 0; attempt < 2 && !ok; attempt++)
	{
		if(attempt == 1)
		{
			if(m_port.GetBaudRate() == 9600)
				break;
			LogDebug("Oscill: no answer at %u baud, retrying at 9600\n", m_port.GetBaudRate());
			m_port.SetBaudRate(9600);
		}

		for(int retry = 0; retry < 2 && !ok; retry++)
		{
			m_port.Drain();
			m_rxBuf.clear();
			if(!SendFrame(BuildConnect()))
				return false;
			if(ReadFrame(reply, CONNECT_TIMEOUT_MS) && IsSuccess(reply))
				ok = true;
		}
	}
	if(!ok)
		return false;

	//The bootloader answers CONNECT with an OBEX Description header (0x05) reading "BOOT" in UTF-16
	static const uint8_t bootUtf16[] = { 0, 'B', 0, 'O', 0, 'O', 0, 'T' };
	if(search(reply.begin(), reply.end(), begin(bootUtf16), end(bootUtf16)) != reply.end())
	{
		LogError("Oscill: probe is in BOOT (bootloader) state, not running its oscilloscope firmware. "
			"Unplug and replug it; do not leave DTR/RTS toggling enabled on the port.\n");
		return false;
	}

	LogDebug("Oscill: connected at %u baud (CONNECT reply: %s)\n", m_port.GetBaudRate(), HexDump(reply).c_str());

	if(wantedBaud != m_port.GetBaudRate())
		ChangeBaudRate(wantedBaud);

	//GET 'C' resets the acquisition engine. Confirmed on hardware: without it, GET 'D' kept returning a stale
	//buffer (all 255) that ignored every setting; after one 'C' the samples follow the input.
	if(!Transact(BuildCommand(CMD_CANCEL), reply))
		LogWarning("Oscill: no reply to the initial cancel\n");

	//Identity registers: 4 ASCII characters each (confirmed on hardware: VNM "Uosc", VSN "3404", VHW "1.25",
	//VSW "1.26"). GET 'F' is only answered in some states (probably by the bootloader), so it isn't used.
	RegValue v;
	string hw;
	if(ReadRegister("VNM", v))
		m_idString = v.AsText();
	if(ReadRegister("VSN", v))
		m_serial = v.AsText();
	if(ReadRegister("VHW", v))
		hw = v.AsText();
	if(ReadRegister("VSW", v))
		m_fwVersion = v.AsText();
	LogNotice("Oscill: %s S/N %s, hardware %s, firmware %s\n",
		m_idString.c_str(), m_serial.c_str(), hw.c_str(), m_fwVersion.c_str());

	//Clock and timebase limits
	for(auto name : { "MCl", "MCh", "MCd" })
	{
		if(ReadRegister(name, v) && v.IsNumeric())
			LogDebug("Oscill: %s = %lld (x10 ps)\n", name, (long long)v.AsInt());
	}
	if( (ReadRegister("MC", v) || ReadRegister("MCd", v)) && v.IsNumeric() && v.u > 0)
		m_mc = v.u;

	//TMl / TMh (25 / 256 on fw 1.26) are NOT the TS limits: the probe accepted TS = 2030000 and sampled at the
	//expected rate. Their meaning is unknown; logged only.
	for(auto name : { "TMl", "TMh", "QSh" })
	{
		if(ReadRegister(name, v) && v.IsNumeric())
			LogDebug("Oscill: %s = %u\n", name, v.u);
	}
	if(ReadRegister("TCh", v) && v.IsNumeric() && v.u > 0)
		m_tcMax = v.u;
	if(ReadRegister("QS", v) && v.IsNumeric() && v.u > 0)
		m_lastDepth = v.u;
	if(ReadRegister("TS", v) && v.IsNumeric() && v.u > 0)
	{
		m_ts = v.u;
		m_sampleRate = (uint64_t)llround(TSToRate(m_ts));
	}

	//Keep the bits we don't control (input filters, trigger hysteresis) as the probe has them
	if(ReadRegister("O1", v) && v.IsNumeric())
		m_o1 = (uint8_t)v.u;
	if(ReadRegister("T1", v) && v.IsNumeric())
		m_t1 = (uint8_t)v.u;
	LogDebug("Oscill: MCU clock %.3f MHz, TS %u (%.6g S/s), %llu samples\n",
		1e9 / (m_mc * MC_UNIT_FS), m_ts, TSToRate(m_ts), (unsigned long long)m_lastDepth);

	return true;
}

/**
	@brief Switches probe and host to a new baud rate (opcode 0x91, recovered from oscilink.dll).

	The probe derives its rate as 1843200 / divisor. We send the request at the current rate, accept whatever comes
	back, switch the host side and verify with GET 'F'. On failure we drop back to 9600 and redo the handshake.
 */
bool OscillOscilloscope::ChangeBaudRate(unsigned int baud)
{
	if(!IsValidBaud(baud))
	{
		LogWarning("Oscill: %u baud is not 1843200/n, staying at %u\n", baud, m_port.GetBaudRate());
		return false;
	}

	//Confirmed on hardware (fw 1.26): the probe acknowledges 0x91 already at the NEW rate, so the reply is garbage
	//at the old one. Send, let it go out, switch the host, discard the garbled ACK, then verify with CONNECT
	//(GET 'F' must not be used: this firmware rejects it).
	CancelPendingAcquisition();
	m_rxBuf.clear();
	SendFrame(BuildSetSpeed(baud));
	usleep(50 * 1000);

	Bytes reply;
	if(m_port.SetBaudRate(baud))
	{
		for(int attempt = 0; attempt < 3; attempt++)
		{
			m_port.Drain();
			m_rxBuf.clear();
			SendFrame(BuildConnect());
			if(ReadFrame(reply, 500, 500) && IsSuccess(reply))
			{
				LogNotice("Oscill: link running at %u baud\n", baud);
				return true;
			}
		}
	}

	//Failed: the probe is most likely already at the new rate. Tell it to go back to 9600 at that rate first,
	//then follow on the host side, so the two sides can't end up mismatched.
	LogWarning("Oscill: baud change to %u failed, reverting to 9600\n", baud);
	SendFrame(BuildSetSpeed(9600));
	usleep(50 * 1000);
	m_port.SetBaudRate(9600);
	m_port.Drain();
	m_rxBuf.clear();
	SendFrame(BuildConnect());
	ReadFrame(reply, 1000);
	return false;
}

bool OscillOscilloscope::SendFrame(const Bytes& frame)
{
	return m_port.Write(frame);
}

/**
	@brief Reads one complete frame from the probe.

	Partial data is kept in m_rxBuf across calls if nothing more arrives before firstByteTimeoutMs expires while
	we're still waiting for the 3 byte frame header. Once the header is in, the body must arrive within bodyTimeoutMs
	or the stream is considered desynchronized and flushed.
 */
bool OscillOscilloscope::ReadFrame(Bytes& frame, int firstByteTimeoutMs, int bodyTimeoutMs)
{
	uint8_t tmp[FRAME_HEADER_LEN];
	while(true)
	{
		//Get the header
		while(m_rxBuf.size() < FRAME_HEADER_LEN)
		{
			size_t want = FRAME_HEADER_LEN - m_rxBuf.size();
			size_t n = m_port.Read(tmp, want, m_rxBuf.empty() ? firstByteTimeoutMs : bodyTimeoutMs);
			if(n == 0)
			{
				if(!m_rxBuf.empty())
				{
					LogWarning("Oscill: truncated frame header, resyncing\n");
					m_rxBuf.clear();
				}
				return false;
			}
			m_rxBuf.insert(m_rxBuf.end(), tmp, tmp + n);
		}

		//Sanity check, drop a byte and try again if this doesn't look like a frame start
		size_t len = FrameLength(&m_rxBuf[0]);
		if(!IsPlausibleOpcode(m_rxBuf[0]) || (len < FRAME_HEADER_LEN) || (len > FRAME_MAX_LEN) )
		{
			LogTrace("Oscill: discarding junk byte %02x\n", m_rxBuf[0]);
			m_rxBuf.erase(m_rxBuf.begin());
			continue;
		}

		//Get the body
		size_t have = m_rxBuf.size();
		if(have < len)
		{
			m_rxBuf.resize(len);
			size_t n = m_port.Read(&m_rxBuf[have], len - have, bodyTimeoutMs);
			if(have + n < len)
			{
				LogWarning("Oscill: frame truncated (%zu of %zu bytes), resyncing\n", have + n, len);
				m_rxBuf.clear();
				m_port.Drain();
				return false;
			}
		}

		frame.swap(m_rxBuf);
		m_rxBuf.clear();
		LogTrace("Oscill RX: %s\n", HexDump(frame).c_str());
		return true;
	}
}

/**
	@brief Sends a request and waits for its reply, aborting any acquisition in flight first
 */
bool OscillOscilloscope::Transact(const Bytes& request, Bytes& reply, int timeoutMs)
{
	if(!m_port.IsOpen())
		return false;

	CancelPendingAcquisition();
	m_rxBuf.clear();

	if(!SendFrame(request))
		return false;

	double deadline = GetTime() + timeoutMs * 1e-3;
	while(true)
	{
		int remaining = (int)((deadline - GetTime()) * 1e3);
		if(remaining <= 0)
			break;
		if(!ReadFrame(reply, remaining))
			break;

		//Ignore a late waveform that raced with the cancel
		if(IsCommandReply(reply, CMD_DATA))
			continue;
		return true;
	}

	LogWarning("Oscill: no reply to request %s\n", HexDump(request, 16).c_str());
	return false;
}

bool OscillOscilloscope::ReadRegister(const string& name, RegValue& value)
{
	//3-letter names are read-only constants ('p' read); 2-letter names are settings (name-only 'q' query)
	Bytes reply;
	auto request = (name.size() >= 3) ? BuildRegisterRead(name) : BuildRegisterQuery(name);
	if(!Transact(request, reply))
		return false;
	if(!IsSuccess(reply))
	{
		LogDebug("Oscill: register %s read refused (%s)\n", name.c_str(), HexDump(reply, 16).c_str());
		return false;
	}
	return ParseRegisterReply(reply, name, value);
}

bool OscillOscilloscope::WriteRegister(const string& name, const Bytes& request)
{
	if(!m_connected)
		return false;

	//Any frame we were holding was captured with the old settings
	m_frameReady = false;

	Bytes reply;
	if(Transact(request, reply) && IsSuccess(reply))
		return true;

	m_regWriteFailures++;
	if(m_regWriteFailures <= 5)
	{
		LogWarning("Oscill: write to register %s failed (sent %s, got %s)\n",
			name.c_str(), HexDump(request, 16).c_str(), HexDump(reply, 16).c_str());
		if(m_regWriteFailures == 5)
			LogWarning("Oscill: suppressing further register write warnings\n");
	}
	return false;
}

bool OscillOscilloscope::WriteReg1(const string& name, uint8_t value)
{
	return WriteRegister(name, BuildRegisterWrite1(name, value));
}

bool OscillOscilloscope::WriteReg2(const string& name, uint16_t value)
{
	return WriteRegister(name, BuildRegisterWrite2(name, value));
}

bool OscillOscilloscope::WriteReg4(const string& name, uint32_t value)
{
	return WriteRegister(name, BuildRegisterWrite4(name, value));
}

/**
	@brief If a GET 'D' is outstanding, cancel it with GET 'C' and swallow whatever comes back
 */
void OscillOscilloscope::CancelPendingAcquisition()
{
	if(!m_requestOutstanding)
		return;
	m_requestOutstanding = false;

	//The hardware trigger never holds a GET 'D' back, so its reply is already on its way. Let it finish instead of
	//sending GET 'C' into it: sending while the probe transmits makes it restart the reply, which desynchronised
	//the stream on real hardware (seen as "a0 00 ba 72 a0 00 ba 72 00 04 44 ..." in ngscopeclient's log).
	Bytes f;
	double deadline = GetTime() + 2.0;
	while(true)
	{
		int remaining = (int)((deadline - GetTime()) * 1e3);
		if(remaining <= 0)
			break;
		if(!ReadFrame(f, remaining))
			continue;
		if(IsCommandReply(f, CMD_DATA))
			return;			//the reply we were waiting for: discard it
		LogWarning("Oscill: unexpected frame while finishing a waveform request: %s\n", HexDump(f, 16).c_str());
		Resync();
		return;
	}

	//Nothing came back: cancel explicitly and expect the ACK (A0 00 03)
	if(!SendFrame(BuildCommand(CMD_CANCEL)))
		return;
	deadline = GetTime() + 1.0;
	while(true)
	{
		int remaining = (int)((deadline - GetTime()) * 1e3);
		if(remaining <= 0 || !ReadFrame(f, remaining))
		{
			LogWarning("Oscill: no ACK for cancel, flushing\n");
			Resync();
			return;
		}
		if(!IsCommandReply(f, CMD_DATA))
			break;
	}
	m_rxBuf.clear();
}

/**
	@brief Throws away everything buffered or still arriving, to get back to a frame boundary
 */
void OscillOscilloscope::Resync()
{
	m_rxBuf.clear();
	m_port.Drain(30);
	m_rxBuf.clear();
}

/**
	@brief Retries the handshake if the probe wasn't answering at construction time (e.g. powered on later)
 */
bool OscillOscilloscope::EnsureConnected()
{
	if(m_connected)
		return true;
	if(!m_port.IsOpen())
		return false;

	m_connected = Connect();
	if(!m_connected)
	{
		LogError("Oscill: probe still not answering, not arming\n");
		return false;
	}
	m_currentRunMode = -1;
	PushAll();
	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Pushing configuration

void OscillOscilloscope::PushAll()
{
	PushCoupling();
	PushVerticalRange();
	PushTimebase();
	PushTriggerLevel();
	PushTriggerSlope();
	PushTriggerDelay();
	PushAutoTimeout();
}

/**
	@brief Auto mode fires on its own after TA. Its power-up value (65280 ticks, ~15.6 s at 50 MHz) makes Auto
	look dead, so set a short timeout.
 */
void OscillOscilloscope::PushAutoTimeout()
{
	double tickS = TIMER_TICK_CLOCKS * m_mc * MC_UNIT_FS / FS_PER_SECOND;
	WriteReg4("TA", (uint32_t)max(1.0, llround(AUTO_TIMEOUT_S / tickS) * 1.0));
}

void OscillOscilloscope::PushCoupling()
{
	uint8_t o1 = m_o1 & ~O1_COUPLING;
	if(m_coupling == OscilloscopeChannel::COUPLE_GND)
		o1 |= O1_GROUND;
	else if(m_coupling == OscilloscopeChannel::COUPLE_AC_1M)
		o1 |= O1_AC;
	if(WriteReg1("O1", o1))
		m_o1 = o1;
}

void OscillOscilloscope::PushVerticalRange()
{
	WriteReg2("V1", (uint16_t)lround(g_voltsPerDiv[m_vrangeIndex] * 1000));

	//Trigger level is stored in ADC codes so it moves with the range
	PushTriggerLevel();
}

void OscillOscilloscope::PushTimebase()
{
	WriteReg4("TS", m_ts);

	//Trigger position is stored in samples so it moves with the timebase
	PushTriggerDelay();
}

void OscillOscilloscope::PushTriggerLevel()
{
	auto et = GetEdgeTrigger();
	if(!et)
		return;
	//S1 = trigger level slider (ScrollTrigCH1), one byte on the ADC code scale
	double code = g_adcZeroCode + et->GetLevel() / GetVoltsPerCode();
	WriteReg1("S1", (uint8_t)lround(max(0.0, min(255.0, code))));
}

void OscillOscilloscope::PushTriggerSlope()
{
	auto et = GetEdgeTrigger();
	if(!et)
		return;

	uint8_t t1 = m_t1 & ~T1_EDGE_MASK;
	switch(et->GetType())
	{
		case EdgeTrigger::EDGE_RISING:
			t1 |= T1_RISING;
			break;
		case EdgeTrigger::EDGE_FALLING:
			t1 |= T1_FALLING;
			break;
		case EdgeTrigger::EDGE_ANY:
			t1 |= T1_RISING | T1_FALLING;
			break;
		default:
			LogWarning("Oscill: unsupported edge type, using rising\n");
			et->SetType(EdgeTrigger::EDGE_RISING);
			t1 |= T1_RISING;
			break;
	}
	if(WriteReg1("T1", t1))
		m_t1 = t1;
}

/**
	@brief Pushes the horizontal trigger position.

	Positive offsets (trigger inside the record) become TC = number of pre-trigger samples, limited to TCh and the
	record length. Negative offsets (record starts after the trigger) become TD, a post-trigger delay in units of
	12000 MCU clocks, with TC = 0.
 */
void OscillOscilloscope::PushTriggerDelay()
{
	int64_t period = GetSamplePeriodFs();
	uint32_t tc = 0;
	uint32_t td = 0;
	if(m_triggerOffset >= 0)
	{
		int64_t pre = m_triggerOffset / period;
		int64_t maxPre = min<int64_t>(m_tcMax, (int64_t)m_lastDepth);
		tc = (uint32_t)max<int64_t>(0, min(pre, maxPre));
	}
	else
	{
		double tickFs = TIMER_TICK_CLOCKS * m_mc * MC_UNIT_FS;
		td = (uint32_t)llround(min(2147483647.0, -m_triggerOffset / tickFs));
	}
	WriteReg2("TC", (uint16_t)tc);
	WriteReg4("TD", td);
}

bool OscillOscilloscope::PushRunMode(uint8_t mode)
{
	if(m_currentRunMode == mode)
		return true;
	if(WriteReg1("RT", mode))
	{
		m_currentRunMode = mode;
		return true;
	}
	//Don't retry on every poll if the probe doesn't accept it
	m_currentRunMode = mode;
	return false;
}

double OscillOscilloscope::GetVoltsPerCode()
{
	return g_voltsPerDiv[m_vrangeIndex] * g_adcDivisions / 256.0 * m_attenuation;
}

int64_t OscillOscilloscope::GetSamplePeriodFs()
{
	//TS/256 MCU clocks per sample
	return (int64_t)llround((double)m_ts * m_mc * MC_UNIT_FS / 256.0);
}

double OscillOscilloscope::TSToRate(uint32_t ts)
{
	return 256.0 * FS_PER_SECOND / ((double)ts * m_mc * MC_UNIT_FS);
}

/**
	@brief TS = sample period in MCU clocks x 256.

	Confirmed on hardware: TS = 2030000 with MC = 1982 (19.82 ns) gave 157.2 us/sample and showed 50 Hz mains hum
	with a 126-sample period (50.5 Hz).
 */
uint32_t OscillOscilloscope::RateToTS(uint64_t rate)
{
	double ts = 256.0 * FS_PER_SECOND / ((double)rate * m_mc * MC_UNIT_FS);
	ts = max((double)m_tsMin, min((double)m_tsMax, ts));
	return (uint32_t)llround(ts);
}

EdgeTrigger* OscillOscilloscope::GetEdgeTrigger()
{
	if(!m_trigger)
		PullTrigger();
	return dynamic_cast<EdgeTrigger*>(m_trigger);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Channel configuration

bool OscillOscilloscope::IsChannelEnabled(size_t i)
{
	return (i == 0) && m_channelEnabled;
}

void OscillOscilloscope::EnableChannel(size_t i)
{
	if(i == 0)
		m_channelEnabled = true;
}

void OscillOscilloscope::DisableChannel(size_t i)
{
	if(i == 0)
		m_channelEnabled = false;
}

OscilloscopeChannel::CouplingType OscillOscilloscope::GetChannelCoupling(size_t /*i*/)
{
	return m_coupling;
}

void OscillOscilloscope::SetChannelCoupling(size_t /*i*/, OscilloscopeChannel::CouplingType type)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	m_coupling = type;
	PushCoupling();
}

vector<OscilloscopeChannel::CouplingType> OscillOscilloscope::GetAvailableCouplings(size_t /*i*/)
{
	return
	{
		OscilloscopeChannel::COUPLE_DC_1M,
		OscilloscopeChannel::COUPLE_AC_1M,
		OscilloscopeChannel::COUPLE_GND
	};
}

double OscillOscilloscope::GetChannelAttenuation(size_t /*i*/)
{
	return m_attenuation;
}

void OscillOscilloscope::SetChannelAttenuation(size_t /*i*/, double atten)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	if(atten <= 0)
		return;

	//Software only: keep the trigger at the same voltage
	m_attenuation = atten;
	PushTriggerLevel();
}

vector<unsigned int> OscillOscilloscope::GetChannelBandwidthLimiters(size_t /*i*/)
{
	return { 0, 3 };
}

unsigned int OscillOscilloscope::GetChannelBandwidthLimit(size_t /*i*/)
{
	return (m_o1 & O1_LOWPASS_3MHZ) ? 3 : 0;
}

//Toggles only the low-pass; the high-pass (ADC mode) is left as it is
void OscillOscilloscope::SetChannelBandwidthLimit(size_t /*i*/, unsigned int limit_mhz)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	bool hp = (m_o1 & O1_HIGHPASS_3KHZ) != 0;
	if(limit_mhz == 0)
		PushFilter(hp ? FILTER_HIGHPASS : FILTER_NONE);
	else
		PushFilter(hp ? FILTER_BANDPASS : FILTER_LOWPASS);
}

bool OscillOscilloscope::IsADCModeConfigurable()
{
	return true;
}

bool OscillOscilloscope::IsADCModePerChannel()
{
	return true;
}

vector<string> OscillOscilloscope::GetADCModeNames(size_t /*channel*/)
{
	return { "Full bandwidth", "Low-pass ~3 MHz (Filt L)", "High-pass ~3 kHz (Filt H)", "Band-pass (Filt L + Filt H)" };
}

size_t OscillOscilloscope::GetADCMode(size_t /*channel*/)
{
	bool lp = (m_o1 & O1_LOWPASS_3MHZ) != 0;
	bool hp = (m_o1 & O1_HIGHPASS_3KHZ) != 0;
	if(lp && hp)
		return FILTER_BANDPASS;
	if(hp)
		return FILTER_HIGHPASS;
	if(lp)
		return FILTER_LOWPASS;
	return FILTER_NONE;
}

void OscillOscilloscope::SetADCMode(size_t /*channel*/, size_t mode)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	if(mode <= FILTER_BANDPASS)
		PushFilter(mode);
}

void OscillOscilloscope::PushFilter(size_t mode)
{
	uint8_t o1 = (m_o1 & ~O1_FILTERS) | g_filterBits[mode];
	if(WriteReg1("O1", o1))
		m_o1 = o1;
}

float OscillOscilloscope::GetChannelVoltageRange(size_t /*i*/, size_t /*stream*/)
{
	return g_voltsPerDiv[m_vrangeIndex] * g_adcDivisions * m_attenuation;
}

void OscillOscilloscope::SetChannelVoltageRange(size_t /*i*/, size_t /*stream*/, float range)
{
	lock_guard<recursive_mutex> lock(m_mutex);

	//Pick the smallest hardware range that fits the request
	double perDiv = range / (g_adcDivisions * m_attenuation);
	size_t idx = g_numVoltRanges - 1;
	for(size_t k=0; k<g_numVoltRanges; k++)
	{
		if(g_voltsPerDiv[k] >= perDiv * 0.999)
		{
			idx = k;
			break;
		}
	}
	m_vrangeIndex = idx;
	PushVerticalRange();
}

OscilloscopeChannel* OscillOscilloscope::GetExternalTrigger()
{
	return nullptr;
}

float OscillOscilloscope::GetChannelOffset(size_t /*i*/, size_t /*stream*/)
{
	return m_offset;
}

void OscillOscilloscope::SetChannelOffset(size_t /*i*/, size_t /*stream*/, float offset)
{
	//Display-only offset. The O1 DAC holds the factory zero calibration and is left untouched.
	m_offset = offset;
}

bool OscillOscilloscope::IsHighRateOffsetCapable(size_t /*i*/)
{
	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Triggering

Oscilloscope::TriggerMode OscillOscilloscope::PollTrigger()
{
	lock_guard<recursive_mutex> lock(m_mutex);

	//Port gone (unplugged / USB reset): stop instead of retrying forever
	if(m_connected && !m_port.IsOpen())
	{
		LogError("Oscill: lost the probe; reconnect it in ngscopeclient\n");
		m_connected = false;
		m_triggerArmed = false;
		m_requestOutstanding = false;
	}

	if(!m_triggerArmed || !m_connected)
		return TRIGGER_MODE_STOP;
	if(m_frameReady)
		return TRIGGER_MODE_TRIGGERED;

	//The probe always free-runs; triggering is done in software on each record (FindEdge)
	uint8_t mode = RUN_FREE;

	//Arm: issue GET 'D'. In Normal mode the probe holds the reply until it triggers.
	if(!m_requestOutstanding)
	{
		PushRunMode(mode);
		m_rxBuf.clear();
		if(!SendFrame(BuildCommand(CMD_DATA)))
			return TRIGGER_MODE_STOP;
		m_requestOutstanding = true;
		m_requestTime = GetTime();
	}

	Bytes f;
	if(ReadFrame(f, POLL_TIMEOUT_MS))
	{
		//In Auto / Normal mode the probe answers at once with an empty body (49 00 03) while nothing has triggered:
		//ask again on the next poll
		WaveformFrame probe;
		if(IsCommandReply(f, CMD_DATA) && !ParseWaveform(f, probe))
		{
			m_requestOutstanding = false;
			return m_autoMode ? TRIGGER_MODE_AUTO : TRIGGER_MODE_WAIT;
		}
		if(IsCommandReply(f, CMD_DATA))
		{
			m_requestOutstanding = false;

			//Software trigger: Normal / Single only accept a record containing the selected edge; Auto shows
			//records without one unaligned; Force accepts anything
			bool found = ApplySoftwareTrigger(probe);
			if(!found && !m_autoMode && !m_forceNext)
				return TRIGGER_MODE_WAIT;

			m_frame.swap(f);
			m_frameReady = true;
			m_forceNext = false;
			return TRIGGER_MODE_TRIGGERED;
		}

		LogWarning("Oscill: unexpected reply to GET 'D': %s\n", HexDump(f, 16).c_str());
		m_requestOutstanding = false;
		Resync();
		return TRIGGER_MODE_WAIT;
	}

	//Free-run and auto must produce data on their own; if not, something got lost
	if( (mode != RUN_NORMAL) && (GetTime() - m_requestTime > FREERUN_WATCHDOG_S) )
	{
		LogWarning("Oscill: no waveform in %.1f s, re-arming\n", FREERUN_WATCHDOG_S);
		CancelPendingAcquisition();
	}

	return m_autoMode ? TRIGGER_MODE_AUTO : TRIGGER_MODE_WAIT;
}

/**
	@brief Looks for the trigger edge in a record and sets m_swTriggerPhase so that the crossing lands at the
	configured trigger offset. Returns false (and a zero phase) if the record has no matching edge.
 */
bool OscillOscilloscope::ApplySoftwareTrigger(const WaveformFrame& wf)
{
	//Runs on the instrument thread: use the cached trigger settings only. ngscopeclient can replace (and delete)
	//the trigger object from the GUI thread at any time without taking our lock.
	m_swTriggerPhase = 0;
	if(wf.samples.size() <= JUNK_SAMPLES + 1)
		return false;

	int edges = m_swTrigEdges;
	int64_t period = GetSamplePeriodFs();
	double level = g_adcZeroCode + m_swTrigLevel / GetVoltsPerCode();
	double target = (double)m_triggerOffset / period;
	//No filtering: narrow (single-sample) pulses are legitimate signals and must be able to trigger
	double pos = FindEdge(&wf.samples[JUNK_SAMPLES], wf.samples.size() - JUNK_SAMPLES,
		level, edges, SW_TRIGGER_HYST_CODES, target);
	LogTrace("Oscill: swtrig level %.1f edges %d target %.1f -> pos %.2f\n", level, edges, target, pos);
	if(pos < 0)
		return false;

	//Sample i is drawn at i * timescale + m_triggerPhase; put the crossing at the trigger offset
	m_swTriggerPhase = m_triggerOffset - (int64_t)llround(pos * period);
	return true;
}

bool OscillOscilloscope::AcquireData()
{
	lock_guard<recursive_mutex> lock(m_mutex);
	if(!m_frameReady)
		return false;
	m_frameReady = false;

	WaveformFrame wf;
	if(!ParseWaveform(m_frame, wf))
	{
		LogWarning("Oscill: malformed waveform frame (%zu bytes)\n", m_frame.size());
		return false;
	}
	LogTrace("Oscill: waveform metadata %s, %zu samples\n", HexDump(wf.metadata).c_str(), wf.samples.size());

	if(m_triggerOneShot)
		m_triggerArmed = false;

	if(!m_channelEnabled)
		return true;

	ChannelsDownloadStarted();
	ChannelsDownloadStatusUpdate(0, InstrumentChannel::DownloadState::DOWNLOAD_IN_PROGRESS, 0.0);

	//Timestamp
	double now = GetTime();
	time_t start = now;
	int64_t fs = (now - start) * FS_PER_SECOND;

	auto chan = GetOscilloscopeChannel(0);
	auto cap = AllocateAnalogWaveform(m_nickname + "." + chan->GetHwname());
	cap->m_timescale = GetSamplePeriodFs();
	cap->m_triggerPhase = m_swTriggerPhase;
	cap->m_startTimestamp = start;
	cap->m_startFemtoseconds = fs;

	//Drop the garbage at the start of every record
	size_t skip = (wf.samples.size() > JUNK_SAMPLES) ? JUNK_SAMPLES : 0;
	size_t n = wf.samples.size() - skip;
	m_lastDepth = n;
	double vpc = GetVoltsPerCode();
	cap->Resize(n);
	cap->PrepareForCpuAccess();
	for(size_t k=0; k<n; k++)
		cap->m_samples[k] = (float)(((int)wf.samples[k + skip] - g_adcZeroCode) * vpc);
	cap->MarkSamplesModifiedFromCpu();

	ChannelsDownloadStatusUpdate(0, InstrumentChannel::DownloadState::DOWNLOAD_FINISHED, 1.0);

	SequenceSet s;
	s[chan] = cap;
	{
		lock_guard<mutex> plock(m_pendingWaveformsMutex);
		m_pendingWaveforms.push_back(s);
	}

	ChannelsDownloadFinished();
	return true;
}

void OscillOscilloscope::Start()
{
	lock_guard<recursive_mutex> lock(m_mutex);
	if(!EnsureConnected())
		return;
	CancelPendingAcquisition();
	m_frameReady = false;
	m_autoMode = false;
	m_forceNext = false;
	m_triggerArmed = true;
	m_triggerOneShot = false;
}

void OscillOscilloscope::StartSingleTrigger()
{
	lock_guard<recursive_mutex> lock(m_mutex);
	if(!EnsureConnected())
		return;
	CancelPendingAcquisition();
	m_frameReady = false;
	m_autoMode = false;
	m_forceNext = false;
	m_triggerArmed = true;
	m_triggerOneShot = true;
}

bool OscillOscilloscope::HasAutoTrigger()
{
	return true;
}

void OscillOscilloscope::StartAutoTrigger()
{
	lock_guard<recursive_mutex> lock(m_mutex);
	if(!EnsureConnected())
		return;
	CancelPendingAcquisition();
	m_frameReady = false;
	m_autoMode = true;
	m_forceNext = false;
	m_triggerArmed = true;
	m_triggerOneShot = false;
}

void OscillOscilloscope::Stop()
{
	lock_guard<recursive_mutex> lock(m_mutex);
	CancelPendingAcquisition();
	m_frameReady = false;
	m_triggerArmed = false;
	m_triggerOneShot = false;
}

void OscillOscilloscope::ForceTrigger()
{
	//One acquisition in free-run mode, i.e. without waiting for a trigger condition
	lock_guard<recursive_mutex> lock(m_mutex);
	if(!EnsureConnected())
		return;
	CancelPendingAcquisition();
	m_frameReady = false;
	m_forceNext = true;
	m_triggerArmed = true;
	m_triggerOneShot = true;
}

bool OscillOscilloscope::IsTriggerArmed()
{
	return m_triggerArmed;
}

vector<string> OscillOscilloscope::GetTriggerTypes()
{
	return { EdgeTrigger::GetTriggerName() };
}

void OscillOscilloscope::PullTrigger()
{
	lock_guard<recursive_mutex> lock(m_mutex);

	//Trigger registers are write-only as far as we know, so the cached trigger object is the source of truth
	if(m_trigger && !dynamic_cast<EdgeTrigger*>(m_trigger))
	{
		delete m_trigger;
		m_trigger = nullptr;
	}
	if(!m_trigger)
	{
		auto et = new EdgeTrigger(this);
		et->SetInput(0, StreamDescriptor(GetOscilloscopeChannel(0), 0), true);
		et->SetLevel(0);
		et->SetType(EdgeTrigger::EDGE_RISING);
		m_trigger = et;
	}
	CacheTriggerSettings();
}

/**
	@brief Copies level and edge from the trigger object into plain members, under m_mutex, for the instrument thread
 */
void OscillOscilloscope::CacheTriggerSettings()
{
	auto et = dynamic_cast<EdgeTrigger*>(m_trigger);
	if(!et)
		return;
	m_swTrigLevel = et->GetLevel();
	switch(et->GetType())
	{
		case EdgeTrigger::EDGE_RISING:		m_swTrigEdges = EDGE_RISING_BIT; break;
		case EdgeTrigger::EDGE_FALLING:		m_swTrigEdges = EDGE_FALLING_BIT; break;
		default:							m_swTrigEdges = EDGE_RISING_BIT | EDGE_FALLING_BIT; break;
	}
}

void OscillOscilloscope::PushTrigger()
{
	lock_guard<recursive_mutex> lock(m_mutex);
	if(!dynamic_cast<EdgeTrigger*>(m_trigger))
	{
		LogWarning("Oscill: only edge triggers are supported\n");
		return;
	}
	CacheTriggerSettings();
	PushTriggerLevel();
	PushTriggerSlope();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Timebase

vector<uint64_t> OscillOscilloscope::GetSampleRatesNonInterleaved()
{
	//1-2-5 steps within what [TMl, TMh] allows, plus the maximum rate itself
	vector<uint64_t> ret;
	double maxRate = TSToRate(m_tsMin);
	double minRate = TSToRate(m_tsMax);
	static const int steps[] = {1, 2, 5};
	for(double decade = 1; decade < 1e10; decade *= 10)
	{
		for(int st : steps)
		{
			double r = st * decade;
			if(r >= minRate && r < maxRate * 0.999)
				ret.push_back((uint64_t)r);
		}
	}
	ret.push_back((uint64_t)llround(maxRate));
	return ret;
}

vector<uint64_t> OscillOscilloscope::GetSampleRatesInterleaved()
{
	return GetSampleRatesNonInterleaved();
}

set<Oscilloscope::InterleaveConflict> OscillOscilloscope::GetInterleaveConflicts()
{
	return {};
}

vector<uint64_t> OscillOscilloscope::GetSampleDepthsNonInterleaved()
{
	return { m_lastDepth };
}

vector<uint64_t> OscillOscilloscope::GetSampleDepthsInterleaved()
{
	return GetSampleDepthsNonInterleaved();
}

uint64_t OscillOscilloscope::GetSampleRate()
{
	return m_sampleRate;
}

uint64_t OscillOscilloscope::GetSampleDepth()
{
	return m_lastDepth;
}

void OscillOscilloscope::SetSampleDepth(uint64_t /*depth*/)
{
	//Fixed record length
}

void OscillOscilloscope::SetSampleRate(uint64_t rate)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	if(rate == 0)
		return;
	m_sampleRate = rate;
	m_ts = RateToTS(rate);
	LogDebug("Oscill: %llu S/s -> TS %u (actual %.6g S/s)\n", (unsigned long long)rate, m_ts, TSToRate(m_ts));
	PushTimebase();
}

void OscillOscilloscope::SetTriggerOffset(int64_t offset)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	m_triggerOffset = offset;
	PushTriggerDelay();
}

int64_t OscillOscilloscope::GetTriggerOffset()
{
	return m_triggerOffset;
}

bool OscillOscilloscope::IsInterleaving()
{
	return false;
}

bool OscillOscilloscope::SetInterleaving(bool /*combine*/)
{
	return false;
}
