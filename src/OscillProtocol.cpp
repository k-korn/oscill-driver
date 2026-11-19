/***********************************************************************************************************************
*                                                                                                                      *
* Oscill DSO1 plugin for ngscopeclient / libscopehal                                                                   *
*                                                                                                                      *
***********************************************************************************************************************/

#include "OscillProtocol.h"

#include <cstdio>

using namespace std;

namespace oscill
{

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Helpers

static void PutU16(Bytes& b, size_t off, uint16_t v)
{
	b[off]   = (v >> 8) & 0xff;
	b[off+1] = v & 0xff;
}

static void AppendU32(Bytes& b, uint32_t v)
{
	b.push_back((v >> 24) & 0xff);
	b.push_back((v >> 16) & 0xff);
	b.push_back((v >> 8) & 0xff);
	b.push_back(v & 0xff);
}

static uint32_t GetU32(const uint8_t* p)
{
	return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

/**
	@brief Assembles opcode + 16-bit total length + body
 */
static Bytes BuildPacket(uint8_t opcode, const Bytes& body)
{
	Bytes b(3);
	b[0] = opcode;
	b.insert(b.end(), body.begin(), body.end());
	PutU16(b, 1, b.size());
	return b;
}

/**
	@brief Variable-length OBEX header: id + 16-bit length (including these 3 bytes) + data
 */
static Bytes BuildHeader(uint8_t headerId, const Bytes& data)
{
	Bytes b(3);
	b[0] = headerId;
	b.insert(b.end(), data.begin(), data.end());
	PutU16(b, 1, b.size());
	return b;
}

static Bytes NameHeader(uint8_t headerId, const string& name)
{
	return BuildHeader(headerId, Bytes(name.begin(), name.end()));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Frame construction
//
// Layouts below come from oscilink.dll v1.04.0003 and are verified on hardware (see docs/Protocol.md).

Bytes BuildConnect(uint16_t maxPacket)
{
	//opcode, length 7, OBEX version 1.0, flags 0, max packet size
	return Bytes{ OP_CONNECT, 0x00, 0x07, 0x10, 0x00, (uint8_t)(maxPacket >> 8), (uint8_t)(maxPacket & 0xff) };
}

Bytes BuildCommand(char cmd)
{
	return BuildPacket(OP_GET, BuildHeader(HDR_COMMAND, Bytes{ (uint8_t)cmd }));
}

//oscilink.dll sends register reads with the PUT opcode
Bytes BuildRegisterRead(const string& name)
{
	return BuildPacket(OP_PUT, NameHeader(HDR_REG_READ, name));
}

/**
	@brief Reads a settings (2-letter, read/write) register: PUT with a name-only 'q' header.

	Confirmed on hardware (fw 1.26): 83 00 08 71 00 05 'RT' -> A0 00 0A 71 00 05 'RT' B1 00. The 'p' read only works
	for the 3-letter read-only registers (MCl, TMh, VNM, ...).
 */
Bytes BuildRegisterQuery(const string& name)
{
	return BuildPacket(OP_PUT, NameHeader(HDR_REG_WRITE, name));
}

Bytes BuildRegisterWrite1(const string& name, uint8_t value)
{
	auto body = NameHeader(HDR_REG_WRITE, name);
	body.push_back(TYPE_U8);
	body.push_back(value);
	return BuildPacket(OP_PUT, body);
}

Bytes BuildRegisterWrite2(const string& name, uint16_t value)
{
	auto body = NameHeader(HDR_REG_WRITE, name);
	body.push_back(TYPE_U16);
	AppendU32(body, value);
	return BuildPacket(OP_PUT, body);
}

Bytes BuildRegisterWrite4(const string& name, uint32_t value)
{
	auto body = NameHeader(HDR_REG_WRITE, name);
	body.push_back(TYPE_U32);
	AppendU32(body, value);
	return BuildPacket(OP_PUT, body);
}

bool IsValidBaud(unsigned int baud)
{
	return (baud >= 7200) && (1843200 % baud == 0) && (1843200 / baud <= 255);
}

Bytes BuildSetSpeed(unsigned int baud)
{
	return BuildPacket(OP_SET_SPEED, Bytes{ (uint8_t)(1843200 / baud) });
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Frame inspection

/**
	@brief Returns true if the byte could be the first byte of a frame coming *from* the probe.

	Response codes in OBEX always have the final bit (0x80) set. We accept the whole 0xA0-0xDF response range
	(success, redirection, client and server errors) so that unexpected error codes don't desync the stream.
 */
bool IsPlausibleOpcode(uint8_t op)
{
	return (op >= 0x90) && (op <= 0xDF);
}

size_t FrameLength(const uint8_t* hdr3)
{
	return ((size_t)hdr3[1] << 8) | hdr3[2];
}

bool IsSuccess(const Bytes& frame)
{
	return (frame.size() >= FRAME_HEADER_LEN) && (frame[0] == OP_SUCCESS);
}

bool IsCommandReply(const Bytes& frame, char cmd)
{
	return IsSuccess(frame) &&
		(frame.size() >= 7) &&
		(frame[3] == HDR_COMMAND) &&
		(frame[6] == (uint8_t)cmd);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Frame decoding

/**
	@brief Decodes a 'D' reply (verified on hardware, fw 1.26):
		A0 LL LL | 72 00 04 44 | 49 HH HH | <4 bytes metadata> <QS samples>
	Returns false for anything else, including the empty reply A0 00 0A 72 00 04 44 49 00 03.
 */
bool ParseWaveform(const Bytes& frame, WaveformFrame& out)
{
	if(!IsCommandReply(frame, CMD_DATA))
		return false;

	//Skip the 'r' header using its own length field
	size_t hlen = ((size_t)frame[4] << 8) | frame[5];
	size_t pos = 3 + hlen;
	if(hlen < 4 || pos + 3 > frame.size())
		return false;

	//End-of-Body header, whose length must cover exactly the rest of the frame
	size_t blen = ((size_t)frame[pos+1] << 8) | frame[pos+2];
	if(frame[pos] != HDR_END_OF_BODY || pos + blen != frame.size())
		return false;
	pos += 3;

	if(frame.size() - pos <= WAVEFORM_METADATA_LEN)
		return false;
	out.metadata.assign(frame.begin() + pos, frame.begin() + pos + WAVEFORM_METADATA_LEN);
	out.samples.assign(frame.begin() + pos + WAVEFORM_METADATA_LEN, frame.end());
	return true;
}

/**
	@brief Extracts the printable ID string from an 'F' reply.

	Tolerates the string being raw after the 'r' header, or wrapped in another OBEX string / byte-sequence header.
 */
string ParseFirmwareString(const Bytes& frame)
{
	if(!IsSuccess(frame))
		return "";

	size_t pos = 3;

	//Skip an echoed 'r' header if present
	if( (frame.size() >= pos + 3) && (frame[pos] == HDR_COMMAND) )
	{
		size_t hlen = ((size_t)frame[pos+1] << 8) | frame[pos+2];
		if(hlen >= 3 && pos + hlen <= frame.size())
			pos += hlen;
	}

	//Unwrap a nested string/byte-sequence header if the length matches exactly
	if( (frame.size() >= pos + 3) && (frame[pos] < 0x80) )
	{
		size_t hlen = ((size_t)frame[pos+1] << 8) | frame[pos+2];
		if(hlen >= 3 && pos + hlen == frame.size())
			pos += 3;
	}

	string s;
	for(size_t i=pos; i<frame.size(); i++)
	{
		uint8_t c = frame[i];
		if(c >= 0x20 && c < 0x7f)
			s += (char)c;
		else if(c == 0 && !s.empty())
			break;
	}

	//Trim
	while(!s.empty() && s.back() == ' ')
		s.pop_back();
	size_t first = s.find_first_not_of(' ');
	return (first == string::npos) ? "" : s.substr(first);
}

/**
	@brief Decodes a register read reply.

	Accepted layouts (the exact one used by the firmware has not been confirmed on hardware):
		A0 LL LL 70|71 hh hh [name] <marker> <value>
		A0 LL LL [name] <marker> <value>
		A0 LL LL <marker> <value>
	A missing/unknown marker means the remainder is treated as a string.
 */
bool ParseRegisterReply(const Bytes& frame, const string& name, RegValue& out)
{
	out = RegValue();
	if(!IsSuccess(frame) || frame.size() <= 3)
		return false;

	size_t pos = 3;
	size_t end = frame.size();

	//Skip the echoed 'p' / 'q' name header. Confirmed on hardware (fw 1.26): the value follows it as a separate header,
	//e.g. A0 00 0E | 70 00 06 'M' 'C' 'l' | F0 00 00 03 20
	if( (frame[pos] == HDR_REG_READ) || (frame[pos] == HDR_REG_WRITE) )
	{
		if(end < pos + 3)
			return false;
		size_t hlen = ((size_t)frame[pos+1] << 8) | frame[pos+2];
		if(hlen < 3 || pos + hlen > end)
			return false;
		bool nameMatches = (hlen - 3 == name.size()) &&
			(0 == name.compare(0, name.size(), (const char*)&frame[pos+3], name.size()));
		if(!nameMatches && (pos + hlen < end))
			return false;
		if(pos + hlen < end)
			pos += hlen;		//value header follows
		else
			pos += 3;			//legacy layout: value inside the header
	}

	//Strip echoed register name
	if( (end - pos >= name.size()) && (0 == name.compare(0, name.size(), (const char*)&frame[pos], name.size())) )
		pos += name.size();

	if(pos >= end)
		return false;

	uint8_t marker = frame[pos];
	if( (marker == TYPE_U8) && (end - pos >= 2) )
	{
		out.kind = RegValue::U8;
		out.u = frame[pos+1];
		out.s = out.u;
		return true;
	}
	if( ((marker == TYPE_U16) || (marker == TYPE_U32)) && (end - pos >= 5) )
	{
		out.kind = (marker == TYPE_U16) ? RegValue::U16 : RegValue::U32;
		out.u = GetU32(&frame[pos+1]);
		out.s = (int32_t)out.u;
		return true;
	}

	//String: skip a string-type OBEX header prefix if present
	if( (frame[pos] < 0x80) && (end - pos >= 3) )
	{
		size_t hlen = ((size_t)frame[pos+1] << 8) | frame[pos+2];
		if(pos + hlen == end)
			pos += 3;
	}
	string s;
	for(size_t i=pos; i<end; i++)
	{
		uint8_t c = frame[i];
		if(c == 0)
			break;
		if(c >= 0x20 && c < 0x7f)
			s += (char)c;
	}
	if(s.empty())
		return false;
	out.kind = RegValue::STRING;
	out.str = s;
	return true;
}

/**
	@brief The identity registers (VNM, VSN, VHW, VSW) hold four ASCII characters packed into a 4-byte value,
	e.g. VSW = F1 31 2E 32 36 = "1.26". Returns that text, or "" if the value isn't printable ASCII.
 */
string RegValue::AsText() const
{
	if(kind == STRING)
		return str;
	if(kind != U32)
		return "";
	string s;
	for(int shift = 24; shift >= 0; shift -= 8)
	{
		char c = (char)((u >> shift) & 0xff);
		if(c == 0)
			continue;
		if(c < 0x20 || c > 0x7e)
			return "";
		s += c;
	}
	return s;
}

double FindEdge(const uint8_t* v, size_t n, double level, int edges, double hyst, double target)
{
	double best = -1;
	double bestDist = 1e30;
	bool armedRise = false;
	bool armedFall = false;
	for(size_t i=0; i<n; i++)
	{
		double x = v[i];
		if(x <= level - hyst)
			armedRise = true;
		if(x >= level + hyst)
			armedFall = true;

		if(i == 0)
			continue;
		double prev = v[i-1];
		double pos = -1;

		if(armedRise && (edges & EDGE_RISING_BIT) && (prev < level) && (x >= level))
		{
			pos = (i - 1) + (level - prev) / (x - prev);
			armedRise = false;
		}
		else if(armedFall && (edges & EDGE_FALLING_BIT) && (prev > level) && (x <= level))
		{
			pos = (i - 1) + (prev - level) / (prev - x);
			armedFall = false;
		}

		//A crossing in the other direction disarms that direction until the signal moves away again
		if( (prev < level) && (x >= level) )
			armedRise = false;
		if( (prev > level) && (x <= level) )
			armedFall = false;

		if(pos >= 0)
		{
			double d = (pos > target) ? (pos - target) : (target - pos);
			if(d < bestDist)
			{
				bestDist = d;
				best = pos;
			}
		}
	}
	return best;
}

string HexDump(const Bytes& b, size_t maxBytes)
{
	string ret;
	char tmp[4];
	for(size_t i=0; i<b.size() && i<maxBytes; i++)
	{
		snprintf(tmp, sizeof(tmp), "%02x ", b[i]);
		ret += tmp;
	}
	if(b.size() > maxBytes)
		ret += "...";
	return ret;
}

}
