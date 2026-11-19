// Unit tests for the Oscill OBEX framing helpers. Byte sequences follow docs/Protocol.md, many captured from a real probe.

#include "OscillProtocol.h"

#include <cstdio>
#include <cstdlib>

using namespace oscill;

static int g_failures = 0;

#define CHECK(cond) do { if(!(cond)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); g_failures++; } } while(0)

int main()
{
	//CONNECT request from the spec
	CHECK(BuildConnect() == (Bytes{0x80, 0x00, 0x07, 0x10, 0x00, 0x10, 0x00}));

	//GET 'D' / 'F' / 'C' from the spec
	CHECK(BuildCommand('D') == (Bytes{0x82, 0x00, 0x07, 0x72, 0x00, 0x04, 0x44}));
	CHECK(BuildCommand('F') == (Bytes{0x82, 0x00, 0x07, 0x72, 0x00, 0x04, 0x46}));
	CHECK(BuildCommand('C') == (Bytes{0x82, 0x00, 0x07, 0x72, 0x00, 0x04, 0x43}));

	//Register writes, layout recovered from oscilink.dll: name header length covers only the name,
	//the value follows as its own OBEX header. Matches the spec example "71 00 05 'O1' B1 00".
	{
		auto b = BuildRegisterWrite1("O1", 0);
		CHECK(b == (Bytes{0x83, 0x00, 0x0a, 0x71, 0x00, 0x05, 'O', '1', 0xB1, 0x00}));
	}
	{
		//spec example: MC = 1250 ps, MC is a 2-byte register -> F0 00 00 04 E2
		auto b = BuildRegisterWrite2("MC", 1250);
		CHECK(b == (Bytes{0x83, 0x00, 0x0d, 0x71, 0x00, 0x05, 'M', 'C', 0xF0, 0x00, 0x00, 0x04, 0xE2}));
	}
	{
		auto b = BuildRegisterWrite4("TD", (uint32_t)-2);
		CHECK(b == (Bytes{0x83, 0x00, 0x0d, 0x71, 0x00, 0x05, 'T', 'D', 0xF1, 0xff, 0xff, 0xff, 0xfe}));
	}
	//Reads use the PUT opcode in oscilink.dll
	CHECK(BuildRegisterRead("VNM") == (Bytes{0x83, 0x00, 0x09, 0x70, 0x00, 0x06, 'V', 'N', 'M'}));

	//Baud rate change: 0x91 + (1843200 / baud)
	CHECK(BuildSetSpeed(9600) == (Bytes{0x91, 0x00, 0x04, 0xC0}));
	CHECK(BuildSetSpeed(115200) == (Bytes{0x91, 0x00, 0x04, 0x10}));
	CHECK(BuildSetSpeed(921600) == (Bytes{0x91, 0x00, 0x04, 0x02}));
	CHECK(IsValidBaud(921600) && IsValidBaud(9600) && !IsValidBaud(250000) && !IsValidBaud(1200));

	//CONNECT reply
	{
		Bytes r{0xA0, 0x00, 0x07, 0x10, 0x00, 0x00, 0x18};
		CHECK(IsSuccess(r));
		CHECK(FrameLength(r.data()) == 7);
		CHECK(IsPlausibleOpcode(r[0]));
		CHECK(!IsPlausibleOpcode(0x44));
	}

	//Long record (QS can go up to QSh = 1788 at <= 1 MS/s): End-of-Body length is 16 bits
	{
		const size_t n = 1788;
		Bytes r{0xA0, 0x00, 0x00, 0x72, 0x00, 0x04, 0x44, 0x49, 0x00, 0x00, 0xE0, 0xE1, 0xE2, 0xE3};
		for(size_t i=0; i<n; i++)
			r.push_back(i & 0xff);
		r[1] = r.size() >> 8;	r[2] = r.size() & 0xff;
		size_t blen = r.size() - 7;
		r[8] = blen >> 8;		r[9] = blen & 0xff;
		CHECK(FrameLength(r.data()) == r.size());
		WaveformFrame wf;
		CHECK(ParseWaveform(r, wf));
		CHECK(wf.metadata.size() == 4 && wf.metadata[0] == 0xE0 && wf.metadata[3] == 0xE3);
		CHECK(wf.samples.size() == n && wf.samples[0] == 0 && wf.samples[n-1] == ((n-1) & 0xff));

		//Inconsistent End-of-Body length or a missing 49 header are rejected
		Bytes bad = r;
		bad[9]--;
		CHECK(!ParseWaveform(bad, wf));
		bad = r;
		bad[7] = 0x48;
		CHECK(!ParseWaveform(bad, wf));
		CHECK(!ParseWaveform(Bytes{0xA0, 0x00, 0x03}, wf));
	}

	//Firmware ID
	{
		std::string id = "Uosc1.25340 41.21";
		Bytes r{0xA0, 0x00, 0x00, 0x72, 0x00, 0x04, 0x46};
		r.insert(r.end(), id.begin(), id.end());
		r[2] = r.size();
		CHECK(ParseFirmwareString(r) == id);

		//Raw string with no echoed header
		Bytes r2{0xA0, 0x00, 0x00};
		r2.insert(r2.end(), id.begin(), id.end());
		r2.push_back(0);
		r2[2] = r2.size();
		CHECK(ParseFirmwareString(r2) == id);
	}

	//Register replies in several plausible layouts
	{
		RegValue v;
		Bytes r{0xA0, 0x00, 0x0d, 0x70, 0x00, 0x0a, 'M', 'C', 'd', 0xF0, 0x00, 0x00, 0x04, 0xE0};
		r[2] = r.size();
		r[5] = r.size() - 3;
		CHECK(ParseRegisterReply(r, "MCd", v));
		CHECK(v.kind == RegValue::U16 && v.u == 1248);

		Bytes r2{0xA0, 0x00, 0x05, 0xB1, 0x07};
		CHECK(ParseRegisterReply(r2, "O1", v));
		CHECK(v.kind == RegValue::U8 && v.u == 7);

		std::string s = "OKB ROSA";
		Bytes r3{0xA0, 0x00, 0x00, 0x70, 0x00, 0x00, 'V', 'N', 'M'};
		r3.insert(r3.end(), s.begin(), s.end());
		r3.push_back(0);
		r3[2] = r3.size();
		r3[5] = r3.size() - 3;
		CHECK(ParseRegisterReply(r3, "VNM", v));
		CHECK(v.kind == RegValue::STRING && v.str == s);

		CHECK(!ParseRegisterReply(Bytes{0xD0, 0x00, 0x03}, "VNM", v));
	}

	//Register read replies captured from a real probe (fw 1.26): echoed 'p' name header, then a value header
	{
		RegValue v;
		Bytes mcl{0xa0, 0x00, 0x0e, 0x70, 0x00, 0x06, 'M', 'C', 'l', 0xf0, 0x00, 0x00, 0x03, 0x20};
		CHECK(ParseRegisterReply(mcl, "MCl", v));
		CHECK(v.kind == RegValue::U16 && v.u == 800);

		Bytes mcd{0xa0, 0x00, 0x0e, 0x70, 0x00, 0x06, 'M', 'C', 'd', 0xf0, 0x00, 0x00, 0x04, 0xe0};
		CHECK(ParseRegisterReply(mcd, "MCd", v) && v.u == 1248);

		Bytes vsw{0xa0, 0x00, 0x0e, 0x70, 0x00, 0x06, 'V', 'S', 'W', 0xf1, 0x31, 0x2e, 0x32, 0x36};
		CHECK(ParseRegisterReply(vsw, "VSW", v));
		CHECK(v.kind == RegValue::U32 && v.AsText() == "1.26");

		Bytes vnm{0xa0, 0x00, 0x0e, 0x70, 0x00, 0x06, 'V', 'N', 'M', 0xf1, 0x55, 0x6f, 0x73, 0x63};
		CHECK(ParseRegisterReply(vnm, "VNM", v) && v.AsText() == "Uosc");

		//Reply for a different register must not be accepted
		CHECK(!ParseRegisterReply(vnm, "VSN", v));

		//Real error reply
		CHECK(!ParseRegisterReply(Bytes{0xd0, 0x00, 0x03}, "MCl", v));

		//Numbers are not text
		Bytes num{0xa0, 0x00, 0x0e, 0x70, 0x00, 0x06, 'T', 'M', 'h', 0xf1, 0x01, 0x00, 0x00, 0x00};
		CHECK(ParseRegisterReply(num, "TMh", v) && v.u == 0x01000000 && v.AsText().empty());
	}

	//Settings register query and replies captured from a real probe (fw 1.26)
	{
		CHECK(BuildRegisterQuery("RT") == (Bytes{0x83, 0x00, 0x08, 0x71, 0x00, 0x05, 'R', 'T'}));
		RegValue v;
		Bytes rt{0xa0, 0x00, 0x0a, 0x71, 0x00, 0x05, 'R', 'T', 0xb1, 0x00};
		CHECK(ParseRegisterReply(rt, "RT", v) && v.kind == RegValue::U8 && v.u == 0);
		Bytes ts{0xa0, 0x00, 0x0d, 0x71, 0x00, 0x05, 'T', 'S', 0xf1, 0x00, 0x00, 0x02, 0x00};
		CHECK(ParseRegisterReply(ts, "TS", v) && v.kind == RegValue::U32 && v.u == 512);
		Bytes mc{0xa0, 0x00, 0x0d, 0x71, 0x00, 0x05, 'M', 'C', 0xf0, 0x00, 0x00, 0x07, 0xbe};	//write echo: 1991 -> 1982
		CHECK(ParseRegisterReply(mc, "MC", v) && v.kind == RegValue::U16 && v.u == 1982);
	}

	//GET 'D' replies captured from a real probe: 4 metadata bytes + QS (254) samples inside an End-of-Body header,
	//or an empty body while nothing has triggered (Auto mode)
	{
		Bytes d{0xa0, 0x01, 0x0c, 0x72, 0x00, 0x04, 0x44, 0x49, 0x01, 0x05, 0x20, 0x00, 0x04, 0x00};
		for(int i=0; i<254; i++)
			d.push_back(128);
		CHECK(d.size() == 0x10c);
		WaveformFrame wf;
		CHECK(ParseWaveform(d, wf));
		CHECK(wf.samples.size() == 254);
		CHECK(wf.metadata.size() == 4 && wf.metadata[0] == 0x20);

		Bytes empty{0xa0, 0x00, 0x0a, 0x72, 0x00, 0x04, 0x44, 0x49, 0x00, 0x03};
		CHECK(IsCommandReply(empty, 'D'));
		CHECK(!ParseWaveform(empty, wf));
	}

	//GET 'F' reply captured from a real probe (first session): string wrapped in an End-of-Body (0x49) header
	{
		Bytes f{0xa0, 0x00, 0x1a, 0x72, 0x00, 0x04, 0x46, 0x49, 0x00, 0x13,
			'U','o','s','c','1','.','2','5','3','4','0','4','1','.','2','1'};
		CHECK(ParseFirmwareString(f) == "Uosc1.2534041.21");
	}

	//Software edge trigger
	{
		auto near = [](double a, double b) { return (a - b < 0.01) && (b - a < 0.01); };

		//Ramp up through 128 between samples 2 and 3 (120 -> 136): crossing at 2.5
		uint8_t up[] = {110, 115, 120, 136, 140, 145};
		CHECK(near(FindEdge(up, 6, 128, EDGE_RISING_BIT, 3, 0), 2.5));
		CHECK(FindEdge(up, 6, 128, EDGE_FALLING_BIT, 3, 0) < 0);
		CHECK(near(FindEdge(up, 6, 128, EDGE_RISING_BIT | EDGE_FALLING_BIT, 3, 0), 2.5));

		//Falling 136 -> 120 between samples 1 and 2
		uint8_t down[] = {140, 136, 120, 110};
		CHECK(near(FindEdge(down, 4, 128, EDGE_FALLING_BIT, 3, 0), 1.5));
		CHECK(FindEdge(down, 4, 128, EDGE_RISING_BIT, 3, 0) < 0);

		//Noise of +-2 codes around the level never arms a 3-code hysteresis
		uint8_t noise[] = {127, 129, 126, 130, 127, 129, 128, 126, 130};
		CHECK(FindEdge(noise, 9, 128, EDGE_RISING_BIT | EDGE_FALLING_BIT, 3, 4) < 0);

		//Two rising crossings (near 1.5 and 7.5): the one closest to the target wins
		uint8_t two[] = {110, 120, 136, 140, 136, 120, 110, 120, 136, 140};
		CHECK(near(FindEdge(two, 10, 128, EDGE_RISING_BIT, 3, 0), 1.5));
		CHECK(near(FindEdge(two, 10, 128, EDGE_RISING_BIT, 3, 9), 7.5));

		//Flat signal far from the level: nothing
		uint8_t flat[] = {200, 201, 199, 200};
		CHECK(FindEdge(flat, 4, 128, EDGE_RISING_BIT | EDGE_FALLING_BIT, 3, 2) < 0);
	}

	if(g_failures)
	{
		printf("%d failure(s)\n", g_failures);
		return 1;
	}
	printf("All protocol tests passed\n");
	return 0;
}
