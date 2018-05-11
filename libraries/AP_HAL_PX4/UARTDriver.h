#pragma once

#include <AP_HAL/utility/RingBuffer.h>

#include "AP_HAL_PX4.h"
#include <systemlib/perf_counter.h>

#if XBEE_TELEM==ENABLED

#define XBEEMAXBUF 350
#define XBEEMAXDATA 250
class Xbee
{
public:
	Xbee() {};
	typedef PX4::PX4UARTDriver S_class; 
	typedef uint8_t(S_class::*call_read)(void);
	typedef uint16_t(S_class::*call_available)(void);
	void set_targ_add(uint64_t* add_list, uint8_t lenth);
	void xbee_init(call_read _read, call_available _available, S_class *_obj);
	//uint16_t get_recv_add();         no use yet
protected:							// for your class inherit from Xbee
	uint16_t pack(const char *s, uint16_t lenth);
	uint16_t pack(const char *s, uint16_t lenth, uint16_t targ_sysid);
	uint8_t decode(void);
	uint16_t data_available();
	uint8_t pack_buf[XBEEMAXBUF];		// get packed here
	uint64_t targ_add;	
	uint64_t targ_add_list[10];			//add number of copter, this should be changed
	uint8_t targ_add_lenth;
private:
	bool operating;
	uint16_t datalenth;
	uint64_t recv_add;
	call_read read;
	call_available available;
	S_class *obj;
	void add_trans(uint8_t* buffer,uint64_t add);
	uint64_t inverse_add_trans(uint8_t * buffer);
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
    void xbee_set_targ_add(uint64_t* add_list, uint8_t lenth);
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

