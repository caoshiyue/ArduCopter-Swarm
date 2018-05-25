#pragma once

#include <AP_HAL/utility/RingBuffer.h>

#include "AP_HAL_PX4.h"
#include <systemlib/perf_counter.h>

#if XBEE_TELEM==ENABLED

#define XBEEMAXBUF 356
#define XBEEMAXDATA 110
class Xbee
{
public:
	Xbee() {};
	typedef uint8_t(PX4::PX4UARTDriver::*call_read)(void);
	typedef uint16_t(PX4::PX4UARTDriver::*call_available)(void);
	void xbee_init(call_read _read, call_available _available,PX4::PX4UARTDriver* _obj);
	void set_targ_add(uint8_t* add_list, uint8_t lenth);
	//uint16_t get_recv_add();
	uint16_t targ_add;
	uint16_t targ_add_list[10];
    	uint8_t targ_add_lenth;
protected:
	uint16_t pack(const char *s, uint16_t lenth);
	uint8_t decode(void);
	uint16_t data_available();
	uint8_t pack_buf[XBEEMAXBUF];
private:
	bool operating;
	uint8_t datalenth;
	uint8_t Frame_ID;
	uint16_t recv_add;
	call_read read;
	call_available available;
	PX4::PX4UARTDriver* obj;
};
#endif

class PX4::PX4UARTDriver : public AP_HAL::UARTDriver,public Xbee
{
public:
    PX4UARTDriver(const char *devpath, const char *perf_name);
    /* PX4 implementations of UARTDriver virtual methods */
    void begin(uint32_t b);
    void begin(uint32_t b, uint16_t rxS, uint16_t txS);
    void end();
    void flush();
    bool is_initialized();
    void set_blocking_writes(bool blocking);
    bool tx_pending();

    /* PX4 implementations of Stream virtual methods */
    uint32_t available() override;
    uint32_t txspace() override;
    int16_t read() override;

    #if XBEE_TELEM==ENABLED
    uint8_t rewrite_read();
    uint16_t rewrite_available();
    uint16_t xbee_available();
    int16_t xbee_read();
    size_t xbee_write(const uint8_t *buffer,size_t size);
    void xbee_set_targ_add(uint8_t* add_list, uint8_t lenth);
	uint16_t xbee_get_recv_add();
    #endif

    /* PX4 implementations of Print virtual methods */
    size_t write(uint8_t c);
    size_t write(const uint8_t *buffer, size_t size);

    void set_device_path(const char *path) {
	    _devpath = path;
    }

    void _timer_tick(void);

    int _get_fd(void) {
	    return _fd;
    }

    void set_flow_control(enum flow_control flow_control);
    enum flow_control get_flow_control(void) { return _flow_control; }

private:
    const char *_devpath;
    int _fd;
    uint32_t _baudrate;
    volatile bool _initialised;
    volatile bool _in_timer;

    bool _nonblocking_writes;

    // we use in-task ring buffers to reduce the system call cost
    // of ::read() and ::write() in the main loop
    ByteBuffer _readbuf{0};
    ByteBuffer _writebuf{0};
    perf_counter_t  _perf_uart;

    int _write_fd(const uint8_t *buf, uint16_t n);
    int _read_fd(uint8_t *buf, uint16_t n);
    uint64_t _first_write_time;
    uint64_t _last_write_time;

    void try_initialise(void);
    uint32_t _last_initialise_attempt_ms;

    uint32_t _os_start_auto_space;
    uint32_t _total_read;
    uint32_t _total_written;
    enum flow_control _flow_control;

    pid_t _uart_owner_pid;

};

