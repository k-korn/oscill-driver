// Unit tests for the Oscill OBEX framing helpers. Byte sequences are taken from docs/oscill-proto.md.

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

	//358 byte waveform reply
	{
		Bytes r{0xA0, 0x01, 0x66, 0x72, 0x00, 0x04, 0x44};
		for(int i=0; i<7; i++)
			r.push_back(0xE0 + i);
		for(int i=0; i<344; i++)
			r.push_back(i & 0xff);
		CHECK(r.size() == 358);
		CHECK(FrameLength(r.data()) == 358);
		WaveformFrame wf;
		CHECK(ParseWaveform(r, wf));
		CHECK(wf.metadata.size() == 7 && wf.metadata[0] == 0xE0);
		CHECK(wf.samples.size() == 344 && wf.samples[0] == 0 && wf.samples[343] == (343 & 0xff));
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

	if(g_failures)
	{
		printf("%d failure(s)\n", g_failures);
		return 1;
	}
	printf("All protocol tests passed\n");
	return 0;
}
