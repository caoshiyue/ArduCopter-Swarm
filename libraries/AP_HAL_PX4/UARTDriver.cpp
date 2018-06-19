#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include "UARTDriver.h"

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <drivers/drv_hrt.h>
#include <assert.h>
#include "GPIO.h"

using namespace PX4;

extern const AP_HAL::HAL& hal;

PX4UARTDriver::PX4UARTDriver(const char *devpath, const char *perf_name) :
	_devpath(devpath),
    _fd(-1),
    _baudrate(57600),
    _initialised(false),
    _in_timer(false),
    _perf_uart(perf_alloc(PC_ELAPSED, perf_name)),
    _os_start_auto_space(-1),
    _flow_control(FLOW_CONTROL_DISABLE)
{

}


extern const AP_HAL::HAL& hal;

/*
  this UART driver maps to a serial device in /dev
 */

void PX4UARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    if (strcmp(_devpath, "/dev/null") == 0) {
        // leave uninitialised
        return;
    }

    uint16_t min_tx_buffer = 1024;
    uint16_t min_rx_buffer = 512;
    if (strcmp(_devpath, "/dev/ttyACM0") == 0) {
        min_tx_buffer = 4096;
        min_rx_buffer = 1024;
    }
    // on PX4 we have enough memory to have a larger transmit and
    // receive buffer for all ports. This means we don't get delays
    // while waiting to write GPS config packets
    if (txS < min_tx_buffer) {
        txS = min_tx_buffer;
    }
    if (rxS < min_rx_buffer) {
        rxS = min_rx_buffer;
    }

    /*
      allocate the read buffer
      we allocate buffers before we successfully open the device as we
      want to allocate in the early stages of boot, and cause minimum
      thrashing of the heap once we are up. The ttyACM0 driver may not
      connect for some time after boot
     */
    if (rxS != _readbuf.get_size()) {
        _initialised = false;
        while (_in_timer) {
            hal.scheduler->delay(1);
        }

        _readbuf.set_size(rxS);
    }

    if (b != 0) {
        _baudrate = b;
    }

    /*
      allocate the write buffer
     */
    if (txS != _writebuf.get_size()) {
        _initialised = false;
        while (_in_timer) {
            hal.scheduler->delay(1);
        }
        _writebuf.set_size(txS);
    }

	if (_fd == -1) {
        _fd = open(_devpath, O_RDWR);
		if (_fd == -1) {
			return;
		}
	}

	if (_baudrate != 0) {
		// set the baud rate
		struct termios t;
		tcgetattr(_fd, &t);
		cfsetspeed(&t, _baudrate);
		// disable LF -> CR/LF
		t.c_oflag &= ~ONLCR;
		tcsetattr(_fd, TCSANOW, &t);

        // separately setup IFLOW if we can. We do this as a 2nd call
        // as if the port has no RTS pin then the tcsetattr() call
        // will fail, and if done as one call then it would fail to
        // set the baudrate.
		tcgetattr(_fd, &t);
		t.c_cflag |= CRTS_IFLOW;
		tcsetattr(_fd, TCSANOW, &t);
	}

    if (_writebuf.get_size() && _readbuf.get_size() && _fd != -1) {
        if (!_initialised) {
            if (strcmp(_devpath, "/dev/ttyACM0") == 0) {
                ((PX4GPIO *)hal.gpio)->set_usb_connected();
            }
            ::printf("initialised %s OK %u %u\n", _devpath,
                     (unsigned)_writebuf.get_size(), (unsigned)_readbuf.get_size());
        }
        _initialised = true;
    }
    _uart_owner_pid = getpid();
}

void PX4UARTDriver::set_flow_control(enum flow_control fcontrol)
{
	if (_fd == -1) {
        return;
    }
    struct termios t;
    tcgetattr(_fd, &t);
    // we already enabled CRTS_IFLOW above, just enable output flow control
    if (fcontrol != FLOW_CONTROL_DISABLE) {
        t.c_cflag |= CRTSCTS;
    } else {
        t.c_cflag &= ~CRTSCTS;
    }
    tcsetattr(_fd, TCSANOW, &t);
    if (fcontrol == FLOW_CONTROL_AUTO) {
        // reset flow control auto state machine
        _total_written = 0;
        _first_write_time = 0;
    }
    _flow_control = fcontrol;
}

void PX4UARTDriver::begin(uint32_t b)
{
	begin(b, 0, 0);
}


/*
  try to initialise the UART. This is used to cope with the way NuttX
  handles /dev/ttyACM0 (the USB port). The port appears in /dev on
  boot, but cannot be opened until a USB cable is connected and the
  host starts the CDCACM communication.
 */
void PX4UARTDriver::try_initialise(void)
{
    if (_initialised) {
        return;
    }
    if ((AP_HAL::millis() - _last_initialise_attempt_ms) < 2000) {
        return;
    }
    _last_initialise_attempt_ms = AP_HAL::millis();
    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_ARMED || !hal.util->get_soft_armed()) {
        begin(0);
    }
}


void PX4UARTDriver::end()
{
    _initialised = false;
    while (_in_timer) hal.scheduler->delay(1);
    if (_fd != -1) {
        close(_fd);
        _fd = -1;
    }

    _readbuf.set_size(0);
    _writebuf.set_size(0);
}

void PX4UARTDriver::flush() {}

bool PX4UARTDriver::is_initialized()
{
    try_initialise();
    return _initialised;
}

void PX4UARTDriver::set_blocking_writes(bool blocking)
{
    _nonblocking_writes = !blocking;
}

bool PX4UARTDriver::tx_pending() { return false; }

/*
  return number of bytes available to be read from the buffer
 */
uint32_t PX4UARTDriver::available()
{
    if (!_initialised) {
        try_initialise();
        return 0;
    }

    return _readbuf.available();
}

/*
  return number of bytes that can be added to the write buffer
 */
uint32_t PX4UARTDriver::txspace()
{
    if (!_initialised) {
        try_initialise();
        return 0;
    }

    return _writebuf.space();
}

/*
  read one byte from the read buffer
 */
int16_t PX4UARTDriver::read()
{
    if (_uart_owner_pid != getpid()){
        return -1;
    }
    if (!_initialised) {
        try_initialise();
        return -1;
    }

    uint8_t byte;
    if (!_readbuf.read_byte(&byte)) {
        return -1;
    }

    return byte;
}

/*
   write one byte to the buffer
 */
size_t PX4UARTDriver::write(uint8_t c)
{
    if (_uart_owner_pid != getpid()){
        return 0;
    }
    if (!_initialised) {
        try_initialise();
        return 0;
    }

    while (_writebuf.space() == 0) {
        if (_nonblocking_writes) {
            return 0;
        }
        hal.scheduler->delay(1);
    }
    return _writebuf.write(&c, 1);
}

/*
  write size bytes to the write buffer
 */
size_t PX4UARTDriver::write(const uint8_t *buffer, size_t size)
{
    if (_uart_owner_pid != getpid()){
        return 0;
    }
	if (!_initialised) {
        try_initialise();
		return 0;
	}

    if (!_nonblocking_writes) {
        /*
          use the per-byte delay loop in write() above for blocking writes
         */
        size_t ret = 0;
        while (size--) {
            if (write(*buffer++) != 1) break;
            ret++;
        }
        return ret;
    }

    return _writebuf.write(buffer, size);
}

#if XBEE_TELEM==ENABLED
uint8_t PX4UARTDriver::rewrite_read()
{
    return (uint8_t)read();
}
uint16_t PX4UARTDriver::rewrite_available()
{
    return (uint16_t)available();
}
uint16_t PX4UARTDriver::xbee_available()
{
    return data_available();
}
int16_t PX4UARTDriver::xbee_read()
{
    return decode();
}
size_t PX4UARTDriver::xbee_write(const uint8_t chan ,const uint8_t *buffer, size_t size)
{
    uint16_t lenth = 0;
    if (chan == 2)
        targ_add = 0xdfdf; //gcs
    else if (chan == 5)
        targ_add = 0xffff; //broadcast
    else
        targ_add = (chan-5 + 0xe0) * 0x0101; //target_add=(targ_sysid+0xe0)*0x0101 sysid: 1 2 3 4
    lenth = pack((const char *)buffer, (uint16_t)size);
    lenth = write(pack_buf, lenth);
    return lenth;
}
#endif

/*
  try writing n bytes, handling an unresponsive port
 */
int PX4UARTDriver::_write_fd(const uint8_t *buf, uint16_t n)
{
    int ret = 0;

    // the FIONWRITE check is to cope with broken O_NONBLOCK behaviour
    // in NuttX on ttyACM0

    // FIONWRITE is also used for auto flow control detection
    // Assume output flow control is not working if:
    //     port is configured for auto flow control
    // and this is not the first write since flow control turned on
    // and no data has been removed from the buffer since flow control turned on
    // and more than .5 seconds elapsed after writing a total of > 5 characters
    //

    int nwrite = 0;

    if (ioctl(_fd, FIONWRITE, (unsigned long)&nwrite) == 0) {
        if (_flow_control == FLOW_CONTROL_AUTO) {
            if (_first_write_time == 0) {
                if (_total_written == 0) {
                    // save the remaining buffer bytes for comparison next write
                    _os_start_auto_space = nwrite;
                }
            } else {
                if (_os_start_auto_space - nwrite + 1 >= _total_written &&
                    (AP_HAL::micros64() - _first_write_time) > 500*1000UL) {
                    // it doesn't look like hw flow control is working
                    ::printf("disabling flow control on %s _total_written=%u\n",
                             _devpath, (unsigned)_total_written);
                    set_flow_control(FLOW_CONTROL_DISABLE);
                }
            }
        }
        if (nwrite > n) {
            nwrite = n;
        }
        if (nwrite > 0) {
            ret = ::write(_fd, buf, nwrite);
        }
    }

    if (ret > 0) {
        _last_write_time = AP_HAL::micros64();
        _total_written += ret;
        if (! _first_write_time && _total_written > 5) {
            _first_write_time = _last_write_time;
        }
        return ret;
    }

    if (AP_HAL::micros64() - _last_write_time > 2000 &&
        _flow_control == FLOW_CONTROL_DISABLE) {
        _last_write_time = AP_HAL::micros64();

        // we haven't done a successful write for 2ms, which means the
        // port is running at less than 500 bytes/sec. Start
        // discarding bytes, even if this is a blocking port. This
        // prevents the ttyACM0 port blocking startup if the endpoint
        // is not connected
        return n;
    }
    return ret;
}

/*
  try reading n bytes, handling an unresponsive port
 */
int PX4UARTDriver::_read_fd(uint8_t *buf, uint16_t n)
{
    int ret = 0;

    // the FIONREAD check is to cope with broken O_NONBLOCK behaviour
    // in NuttX on ttyACM0
    int nread = 0;
    if (ioctl(_fd, FIONREAD, (unsigned long)&nread) == 0) {
        if (nread > n) {
            nread = n;
        }
        if (nread > 0) {
            ret = ::read(_fd, buf, nread);
        }
    }
    if (ret > 0) {
        _total_read += ret;
    }
    return ret;
}


/*
  push any pending bytes to/from the serial port. This is called at
  1kHz in the timer thread. Doing it this way reduces the system call
  overhead in the main task enormously.
 */
void PX4UARTDriver::_timer_tick(void)
{
    int ret;
    uint32_t n;

    if (!_initialised) return;

    // don't try IO on a disconnected USB port
    if (strcmp(_devpath, "/dev/ttyACM0") == 0 && !hal.gpio->usb_connected()) {
        return;
    }

    _in_timer = true;

    // write any pending bytes
    n = _writebuf.available();
    if (n > 0) {
        ByteBuffer::IoVec vec[2];
        perf_begin(_perf_uart);
        const auto n_vec = _writebuf.peekiovec(vec, n);
        for (int i = 0; i < n_vec; i++) {
            ret = _write_fd(vec[i].data, (uint16_t)vec[i].len);
            if (ret < 0) {
                break;
            }
            _writebuf.advance(ret);

            /* We wrote less than we asked for, stop */
            if ((unsigned)ret != vec[i].len) {
                break;
            }
        }
        perf_end(_perf_uart);
    }

    // try to fill the read buffer
    ByteBuffer::IoVec vec[2];

    perf_begin(_perf_uart);
    const auto n_vec = _readbuf.reserve(vec, _readbuf.space());
    for (int i = 0; i < n_vec; i++) {
        ret = _read_fd(vec[i].data, vec[i].len);
        if (ret < 0) {
            break;
        }
        _readbuf.commit((unsigned)ret);

        /* stop reading as we read less than we asked for */
        if ((unsigned)ret < vec[i].len) {
            break;
        }
    }
    perf_end(_perf_uart);

    _in_timer = false;
}

#if XBEE_TELEM==ENABLED

void Xbee::xbee_init(call_read _read, call_available _available,PX4::PX4UARTDriver* _obj)
{
    operating = false;
    datalenth = 0;
    Frame_ID = 0;
    targ_add = 0xdfdf;
    read = _read;
    available = _available;
    obj=_obj;
}

//start-delimeter Length Frame_type Frame_ID Address Option-byte Data Checksum
//7E			  __ __	 01		    __		 __ __	 00			 ____ 81
uint16_t Xbee::pack(const char * data, uint16_t lenth)// lenth of data
{
	uint16_t pack_lenth;
	uint8_t i = 0;
	uint8_t check;
        memset(pack_buf, 0, sizeof(pack_buf));
	for (i = 0;i < ((lenth - 1) / XBEEMAXDATA) + 1;i++)
	{
		pack_lenth = 0;
		if ((lenth - XBEEMAXDATA * i) >= XBEEMAXDATA)
			pack_lenth = XBEEMAXDATA;
		else
			pack_lenth = lenth - XBEEMAXDATA * i;
		pack_buf[0 + (XBEEMAXDATA + 9) * i] = 0x7e;
		pack_lenth += 5;
		pack_buf[1 + (XBEEMAXDATA + 9) * i] = pack_lenth >> 8;
		pack_buf[2 + (XBEEMAXDATA + 9) * i] = pack_lenth & 0xff;
		pack_buf[3 + (XBEEMAXDATA + 9) * i] = 1;
                pack_buf[4 + (XBEEMAXDATA + 9) * i] = 0;
		pack_buf[5 + (XBEEMAXDATA + 9) * i] = targ_add >> 8;
		pack_buf[6 + (XBEEMAXDATA + 9) * i] = targ_add & 0xff;
		pack_buf[7 + (XBEEMAXDATA + 9) * i] = 0;
		memcpy(&pack_buf[8 + (XBEEMAXDATA + 9) * i], (data + XBEEMAXDATA * i), pack_lenth - 5);
		check = 0;
		for (int u = 3;u < pack_lenth + 3;u++)
			check += pack_buf[u + (XBEEMAXDATA + 9) * i];
		pack_buf[pack_lenth + 3 + (XBEEMAXDATA + 9) * i] = 0xFF - check;
	}
	return lenth + 9 * i;
}

//start-delimeter Length Frame_type Address RSSI Option-byte Data Checksum
//7E			  __ __	 81		    __ __	__   00			 ____ 81
uint8_t Xbee::decode(void)
{
        static uint8_t cursor = 0;
        if (!operating)
                return -1;
        else
        {
                uint8_t byte = 0;
                byte = obj->read();
                cursor++;
                if (cursor == datalenth)
                {
                        operating = false;
                        cursor = 0;
                        obj->read();
                }
                return byte;
        }
}
uint16_t Xbee::data_available()
{
        if (!operating)
        {
                uint8_t byte[8] = { 0 };
                while (obj->available() > 8)
                {
                        byte[0] = obj->read();
                        if (byte[0] == 0x7E)
                        {
                                for (int i = 1;i < 4;i++)
                                        byte[i] = obj->read();
                                if (byte[3] == 0x81)
                                {
                                        for (int i = 4;i < 8;i++)
                                                byte[i] = obj->read();
                                        recv_add = byte[4] * 256 + byte[5];
                                        datalenth= byte[1] * 256 + byte[2] - 5;
                                        operating = true;
                                        return obj->available();
                                }
                        }
                }
                return 0;
        }
        else
                return obj->available();
}
#endif //XBEE_TELEM

#endif
