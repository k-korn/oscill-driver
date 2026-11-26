/***********************************************************************************************************************
*                                                                                                                      *
* Oscill DSO1 plugin for ngscopeclient / libscopehal                                                                   *
*                                                                                                                      *
* OBEX-style framing helpers for the Oscill DSO1 pen oscilloscope. This file is intentionally free of any libscopehal *
* dependency so it can be unit tested on its own.                                                                      *
*                                                                                                                      *
***********************************************************************************************************************/

#ifndef OscillProtocol_h
#define OscillProtocol_h

#include <cstdint>
#include <string>
#include <vector>

namespace oscill
{

typedef std::vector<uint8_t> Bytes;

//OBEX opcodes (standard OBEX numbering; Oscill extensions per the vendor's protocol document)
enum Opcode : uint8_t
{
	OP_CONNECT		= 0x80,
	OP_DISCONNECT	= 0x81,
	OP_PUT			= 0x82,		//final PUT: register writes, 'C' (calibrate)
	OP_GET			= 0x83,		//final GET: property / register reads, 'D' (digitize)
	OP_SET_SPEED	= 0x91,		//Oscill extension: body = one byte, 1843200 / baud
	OP_REPEAT		= 0x92,		//Oscill extension: repeat the last response
	OP_ABORT		= 0xFF,		//stop the current operation (e.g. a roll-mode stream)
	OP_CONTINUE		= 0x90,		//response: part of a longer reply
	OP_SUCCESS		= 0xA0,		//response
	OP_ERROR		= 0xD0		//response: internal server error (e.g. corrupted request)
};

//Oscill-specific header IDs
enum HeaderId : uint8_t
{
	HDR_REG_READ	= 0x70,		//'p'
	HDR_REG_WRITE	= 0x71,		//'q'
	HDR_COMMAND		= 0x72		//'r'
};

/*
	Register values travel as a separate OBEX header following the 'p'/'q' name header (confirmed by disassembling
	oscilink.dll v1.04.0003, function at 0x1103f720):
		1-byte register:  B1 vv
		2-byte register:  F0 00 00 hi lo     (4-byte OBEX quantity, zero extended)
		4-byte register:  F1 b3 b2 b1 b0
 */
enum TypeMarker : uint8_t
{
	TYPE_U8			= 0xB1,
	TYPE_U16		= 0xF0,
	TYPE_U32		= 0xF1
};

//Core commands carried in the 'r' header
enum Command : char
{
	CMD_DATA		= 'D',		//digitize (GET): the reply comes when a fresh record has been acquired
	CMD_CALIBRATE	= 'C'		//self-calibration of zero and trigger level (PUT)
	//'F' is the vendor's firmware-load command: never sent by this driver
};

//OBEX End-of-Body header: carries the waveform data
static const uint8_t HDR_END_OF_BODY = 0x49;

///@brief Smallest legal frame (opcode + 16-bit length)
static const size_t FRAME_HEADER_LEN = 3;

///@brief Upper bound on frame size we accept. Records go up to QSh (1788 seen) samples; 4 kB is plenty.
static const size_t FRAME_MAX_LEN = 4096;

///@brief Record length (QS) at power-up
static const size_t DEFAULT_RECORD_LEN = 254;

///@brief Metadata bytes at the start of the End-of-Body data, before the samples
static const size_t WAVEFORM_METADATA_LEN = 4;

///@brief A value read from / written to a register
struct RegValue
{
	enum Kind { NONE, U8, U16, U32, STRING } kind = NONE;
	uint32_t u = 0;
	int32_t s = 0;
	std::string str;

	bool IsNumeric() const
	{ return (kind == U8) || (kind == U16) || (kind == U32); }

	int64_t AsInt() const
	{ return (int64_t)u; }

	std::string AsText() const;
};

///@brief A decoded 'D' response
struct WaveformFrame
{
	Bytes metadata;
	Bytes samples;
};

//Frame construction
Bytes BuildConnect(uint16_t maxPacket = 0x1000);
Bytes BuildCommand(char cmd);
Bytes BuildAbort();
Bytes BuildRegisterRead(const std::string& name);
Bytes BuildRegisterQuery(const std::string& name);
Bytes BuildRegisterWrite1(const std::string& name, uint8_t value);
Bytes BuildRegisterWrite2(const std::string& name, uint16_t value);
Bytes BuildRegisterWrite4(const std::string& name, uint32_t value);
Bytes BuildSetSpeed(unsigned int baud);
bool IsValidBaud(unsigned int baud);

//Frame inspection
bool IsPlausibleOpcode(uint8_t op);
size_t FrameLength(const uint8_t* hdr3);
bool IsSuccess(const Bytes& frame);
bool IsCommandReply(const Bytes& frame, char cmd);

//Frame decoding
bool ParseWaveform(const Bytes& frame, WaveformFrame& out);
bool ParseRegisterReply(const Bytes& frame, const std::string& name, RegValue& out);

std::string HexDump(const Bytes& b, size_t maxBytes = 64);

/**
	@brief Software edge trigger (the probe's hardware trigger does not gate captures on fw 1.26).

	Finds crossings of `level` (in ADC codes) with hysteresis: a rising edge needs the signal at or below
	level - hyst before it reaches level, a falling edge the mirror image. The crossing position is interpolated
	between the two samples around it. Of all matching crossings, the one closest to `target` (a sample index,
	normally the configured pre-trigger position) is returned.

	@param edges	bit 0 = rising, bit 1 = falling
	@return Fractional sample index of the chosen crossing, or a negative value if there is none
 */
enum { EDGE_RISING_BIT = 1, EDGE_FALLING_BIT = 2 };
double FindEdge(const uint8_t* v, size_t n, double level, int edges, double hyst, double target);

}

#endif
