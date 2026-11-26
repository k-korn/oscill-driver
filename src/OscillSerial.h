/***********************************************************************************************************************
*                                                                                                                      *
* Oscill DSO1 plugin for ngscopeclient / libscopehal                                                                   *
*                                                                                                                      *
***********************************************************************************************************************/

#ifndef OscillSerial_h
#define OscillSerial_h

#include "OscillProtocol.h"

class SCPITransport;

/**
	@brief Timed raw I/O on top of the serial port owned by libscopehal's "uart" transport.

	libscopehal's UART class performs fully blocking reads on Linux (VMIN=1, SetTimeouts() is a no-op), which would
	hang the instrument thread forever if the probe stops answering. We borrow the file descriptor from the transport
	and do our own poll()-based reads with explicit timeouts, plus baud rate changes.
 */
class OscillSerial
{
public:
	OscillSerial();

	bool Attach(SCPITransport* transport);
	bool IsOpen() const
	{ return m_fd >= 0; }

	bool SetBaudRate(unsigned int baud);
	unsigned int GetBaudRate() const
	{ return m_baud; }

	bool Write(const oscill::Bytes& data);

	/**
		@brief Reads up to len bytes, returning early only on timeout.
		@return Number of bytes actually read
	 */
	size_t Read(uint8_t* buf, size_t len, int timeoutMs);

	///@brief Discards everything in the kernel RX buffer plus anything that arrives within quietMs
	size_t Drain(int quietMs = 20);		//returns the number of bytes thrown away

	void SetModemLines(bool high);
	std::string DescribeState();

protected:
	void Fail(const char* what, int err);

	int m_fd;
	unsigned int m_baud;
};

#endif
