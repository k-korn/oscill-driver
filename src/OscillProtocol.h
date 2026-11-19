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

//OBEX opcodes used by the probe
enum Opcode : uint8_t
{
	OP_CONNECT		= 0x80,
	OP_GET			= 0x82,
	OP_GET_CONT		= 0x02,
	OP_PUT			= 0x83,
	OP_SET_SPEED	= 0x91,		//body: one byte = 1843200 / baud (from oscilink.dll)
	OP_ABORT		= 0x92,
	OP_SUCCESS		= 0xA0,
	OP_ERROR		= 0xD0
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
	CMD_DATA		= 'D',		//fetch a waveform (blocks until trigger in Normal/Wait mode)
	CMD_FIRMWARE	= 'F',		//firmware / ID string
	CMD_CANCEL		= 'C'		//cancel a pending 'D'
};

///@brief Smallest legal frame (opcode + 16-bit length)
static const size_t FRAME_HEADER_LEN = 3;

///@brief Upper bound on frame size we accept. The probe negotiates 0x0018 / 0x1000 in CONNECT; 4 kB is plenty.
static const size_t FRAME_MAX_LEN = 4096;

///@brief Nominal number of ADC samples in one 'D' reply
static const size_t NOMINAL_SAMPLE_COUNT = 344;

///@brief Number of metadata bytes between the 'r'/'D' header and the samples
static const size_t WAVEFORM_METADATA_LEN = 7;

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
std::string ParseFirmwareString(const Bytes& frame);
bool ParseRegisterReply(const Bytes& frame, const std::string& name, RegValue& out);

std::string HexDump(const Bytes& b, size_t maxBytes = 64);

}

#endif
