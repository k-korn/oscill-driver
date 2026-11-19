/***********************************************************************************************************************
*                                                                                                                      *
* Oscill DSO1 plugin for ngscopeclient / libscopehal                                                                   *
*                                                                                                                      *
***********************************************************************************************************************/

#include "scopehal.h"
#include "SCPIUARTTransport.h"
#include "OscillSerial.h"

#include <cerrno>
#include <chrono>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

using namespace std;

/**
	@brief Gives us the UART member of SCPIUARTTransport without modifying libscopehal.

	Forming a pointer-to-member through a derived class is the standard-conforming way to reach a protected member
	of an object we didn't create; the class itself is never instantiated.
 */
class OscillUartAccess : public SCPIUARTTransport
{
public:
	static UART& Get(SCPIUARTTransport* t)
	{
		UART SCPIUARTTransport::* p = &OscillUartAccess::m_uart;
		return t->*p;
	}
};

static speed_t BaudToSpeed(unsigned int baud)
{
	switch(baud)
	{
		case 9600:		return B9600;
		case 19200:		return B19200;
		case 38400:		return B38400;
		case 57600:		return B57600;
		case 115200:	return B115200;
		case 230400:	return B230400;
		case 460800:	return B460800;
		case 921600:	return B921600;
		default:		return B0;
	}
}

static int64_t NowMs()
{
	return chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now().time_since_epoch()).count();
}

OscillSerial::OscillSerial()
	: m_fd(-1)
	, m_baud(0)
{
}

bool OscillSerial::Attach(SCPITransport* transport)
{
	auto uart = dynamic_cast<SCPIUARTTransport*>(transport);
	if(!uart)
	{
		LogError("Oscill: this driver requires the \"uart\" transport (e.g. /dev/ttyUSB0:9600)\n");
		return false;
	}
	if(!uart->IsConnected())
	{
		LogError("Oscill: serial port is not open\n");
		return false;
	}

	m_fd = OscillUartAccess::Get(uart).GetHandle();

	//Hold DTR and RTS low. With these lines toggling on open/close the probe was observed to reset into its
	//bootloader ("BOOT" in the CONNECT reply); with them held low it stays in the oscilloscope firmware.
	int lines = TIOCM_DTR | TIOCM_RTS;
	if(0 != ioctl(m_fd, TIOCMBIC, &lines))
		LogDebug("Oscill: could not clear DTR/RTS (%s)\n", strerror(errno));

	//Remember the baud rate the user asked for in the connection string
	auto cs = uart->GetConnectionString();
	unsigned int baud = 9600;
	auto colon = cs.find(':');
	if(colon != string::npos)
		baud = strtoul(cs.c_str() + colon + 1, nullptr, 10);
	return SetBaudRate(baud ? baud : 9600);
}

bool OscillSerial::SetBaudRate(unsigned int baud)
{
	if(m_fd < 0)
		return false;

	speed_t sp = BaudToSpeed(baud);
	if(sp == B0)
	{
		LogError("Oscill: unsupported baud rate %u\n", baud);
		return false;
	}

	termios t;
	if(0 != tcgetattr(m_fd, &t))
	{
		LogError("Oscill: tcgetattr failed (%s)\n", strerror(errno));
		return false;
	}
	cfmakeraw(&t);
	t.c_cflag |= CLOCAL | CREAD;
	t.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS | HUPCL);	//no HUPCL: don't drop DTR when the port closes
	t.c_iflag &= ~(IXON | IXOFF | IXANY);
	t.c_cc[VMIN] = 1;
	t.c_cc[VTIME] = 0;
	cfsetispeed(&t, sp);
	cfsetospeed(&t, sp);
	if(0 != tcsetattr(m_fd, TCSANOW, &t))
	{
		LogError("Oscill: tcsetattr failed (%s)\n", strerror(errno));
		return false;
	}

	m_baud = baud;
	LogDebug("Oscill: serial port set to %u 8N1\n", baud);
	return true;
}

bool OscillSerial::Write(const oscill::Bytes& data)
{
	if(m_fd < 0)
		return false;

	LogTrace("Oscill TX: %s\n", oscill::HexDump(data).c_str());

	size_t pos = 0;
	while(pos < data.size())
	{
		ssize_t n = write(m_fd, data.data() + pos, data.size() - pos);
		if(n < 0)
		{
			if(errno == EINTR || errno == EAGAIN)
				continue;
			LogError("Oscill: write failed (%s)\n", strerror(errno));
			return false;
		}
		pos += n;
	}
	return true;
}

size_t OscillSerial::Read(uint8_t* buf, size_t len, int timeoutMs)
{
	if(m_fd < 0)
		return 0;

	size_t pos = 0;
	int64_t deadline = NowMs() + timeoutMs;
	while(pos < len)
	{
		int remaining = (int)(deadline - NowMs());
		if(remaining < 0)
			remaining = 0;

		pollfd pfd;
		pfd.fd = m_fd;
		pfd.events = POLLIN;
		pfd.revents = 0;
		int r = poll(&pfd, 1, remaining);
		if(r < 0)
		{
			if(errno == EINTR)
				continue;
			LogError("Oscill: poll failed (%s)\n", strerror(errno));
			break;
		}
		if(r == 0)
			break;		//timeout
		if(pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
		{
			LogError("Oscill: serial port error / hangup\n");
			break;
		}

		ssize_t n = read(m_fd, buf + pos, len - pos);
		if(n < 0)
		{
			if(errno == EINTR || errno == EAGAIN)
				continue;
			LogError("Oscill: read failed (%s)\n", strerror(errno));
			break;
		}
		if(n == 0)
			break;
		pos += n;
	}
	return pos;
}

void OscillSerial::Drain(int quietMs)
{
	if(m_fd < 0)
		return;
	tcflush(m_fd, TCIFLUSH);
	uint8_t junk[256];
	while(Read(junk, sizeof(junk), quietMs) > 0)
	{}
}
