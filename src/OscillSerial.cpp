/***********************************************************************************************************************
*                                                                                                                      *
* Oscill DSO1 plugin for ngscopeclient / libscopehal                                                                   *
*                                                                                                                      *
***********************************************************************************************************************/

#include "scopehal.h"
#include "SCPIUARTTransport.h"
#include "OscillSerial.h"

#include <cerrno>
#include <cstring>
#include <dirent.h>
#include <sys/stat.h>
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

	//Set the baud rate the user asked for in the connection string
	auto cs = uart->GetConnectionString();
	unsigned int baud = 9600;
	auto colon = cs.find(':');
	if(colon != string::npos)
		baud = strtoul(cs.c_str() + colon + 1, nullptr, 10);
	if(!SetBaudRate(baud ? baud : 9600))
		return false;

	//Then hold DTR and RTS low (with them toggling the probe was seen to reset into its bootloader). This must come
	//after setting the baud rate: libscopehal leaves the port at B0 (hang up, lines low), and leaving B0 makes the
	//tty driver raise both lines again.
	SetModemLines(false);
	return true;
}

/**
	@brief Drives DTR and RTS together
 */
void OscillSerial::SetModemLines(bool high)
{
	if(m_fd < 0)
		return;
	int lines = TIOCM_DTR | TIOCM_RTS;
	if(0 != ioctl(m_fd, high ? TIOCMBIS : TIOCMBIC, &lines))
		LogDebug("Oscill: could not %s DTR/RTS (%s)\n", high ? "set" : "clear", strerror(errno));
}

/**
	@brief One line of host-side port state, for diagnosing a probe that doesn't answer
 */
string OscillSerial::DescribeState()
{
	if(m_fd < 0)
		return "port closed";

	int st = 0, outq = -1, inq = -1;
	ioctl(m_fd, TIOCMGET, &st);
	ioctl(m_fd, TIOCOUTQ, &outq);
	ioctl(m_fd, TIOCINQ, &inq);
	termios t;
	memset(&t, 0, sizeof(t));
	tcgetattr(m_fd, &t);

	char path[256] = {0};
	char link[64];
	snprintf(link, sizeof(link), "/proc/self/fd/%d", m_fd);
	ssize_t n = readlink(link, path, sizeof(path) - 1);
	if(n > 0)
		path[n] = 0;

	char buf[512];
	snprintf(buf, sizeof(buf), "fd %d -> %s, DTR=%d RTS=%d CTS=%d DSR=%d, out queue %d, in queue %d, "
		"cflag 0x%x (CBAUD 0x%x), ospeed %u",
		m_fd, path, !!(st & TIOCM_DTR), !!(st & TIOCM_RTS), !!(st & TIOCM_CTS), !!(st & TIOCM_DSR), outq, inq,
		(unsigned)t.c_cflag, (unsigned)(t.c_cflag & CBAUD), (unsigned)cfgetospeed(&t));
	string ret = buf;

	//Anyone else holding the same device? (only our own user's processes are visible)
	struct stat me;
	if( (n > 0) && (0 == stat(path, &me)) )
	{
		if(auto proc = opendir("/proc"))
		{
			while(auto e = readdir(proc))
			{
				int pid = atoi(e->d_name);
				if( (pid <= 0) || (pid == getpid()) )
					continue;
				string fddir = string("/proc/") + e->d_name + "/fd";
				if(auto fds = opendir(fddir.c_str()))
				{
					while(auto f = readdir(fds))
					{
						struct stat st2;
						if( (0 == stat((fddir + "/" + f->d_name).c_str(), &st2)) && S_ISCHR(st2.st_mode) &&
							(st2.st_rdev == me.st_rdev) )
						{
							string comm;
							if(FILE* fp = fopen((string("/proc/") + e->d_name + "/comm").c_str(), "r"))
							{
								char c[64] = {0};
								if(fgets(c, sizeof(c), fp))
									comm = c;
								fclose(fp);
								if(!comm.empty() && comm.back() == '\n')
									comm.pop_back();
							}
							ret += "; also open in PID " + string(e->d_name) + " (" + comm + ")";
							break;
						}
					}
					closedir(fds);
				}
			}
			closedir(proc);
		}
	}
	return ret;
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
			Fail("write failed", errno);
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
			Fail("poll failed", errno);
			break;
		}
		if(r == 0)
			break;		//timeout
		if(pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
		{
			Fail("serial port error / hangup (probe unplugged or reset?)", 0);
			break;
		}

		ssize_t n = read(m_fd, buf + pos, len - pos);
		if(n < 0)
		{
			if(errno == EINTR || errno == EAGAIN)
				continue;
			Fail("read failed", errno);
			break;
		}
		if(n == 0)
			break;
		pos += n;
	}
	return pos;
}

size_t OscillSerial::Drain(int quietMs)
{
	if(m_fd < 0)
		return 0;
	int inq = 0;
	ioctl(m_fd, TIOCINQ, &inq);
	tcflush(m_fd, TCIFLUSH);
	size_t total = (inq > 0) ? inq : 0;
	uint8_t junk[256];
	size_t n;
	while( (n = Read(junk, sizeof(junk), quietMs)) > 0)
	{
		LogTrace("Oscill: drained %s\n", oscill::HexDump(oscill::Bytes(junk, junk + n)).c_str());
		total += n;
	}
	return total;
}

/**
	@brief Marks the port dead after an unrecoverable error (e.g. the USB device went away), logging only once.

	The fd belongs to libscopehal's transport, so it is not closed here; IsOpen() just returns false from now on and
	every later read / write fails immediately instead of spinning on errors.
 */
void OscillSerial::Fail(const char* what, int err)
{
	if(m_fd < 0)
		return;
	if(err)
		LogError("Oscill: %s (%s); giving up on the serial port\n", what, strerror(err));
	else
		LogError("Oscill: %s; giving up on the serial port\n", what);
	m_fd = -1;
}
