# I2C slave interface in Micropython for RP2040.

An I2C slave implemented Micropython for RP2040
(i.e., the [rp2 port](https://micropython.org/download/?port=rp2)),
with a high-level general-purpose API.

The implementation is optimized for fast response times and minimal RAM usage.


# Usage

The file `test_i2c_slave.py` contains a minimal example, which is small enough
to be included here:

~~~
import rp2040_i2c_slave


def test_slave(addr: int, bus=0, sda=0, scl=1):
    # Instantiate the slave
    s_i2c = rp2040_i2c_slave.I2C_Slave(bus, sda=sda, scl=scl, address=addr)

    # Implement an I2C-attached memory of 256 bytes
    MEM_SIZE = 256
    data_buf = bytearray(MEM_SIZE)

    while True:
        # Check the bus state
        state = s_i2c.poll()

        if state == rp2040_i2c_slave.POLL_RECEIVE:
            # Master has initiated a write; store the data in memory.
            _ = s_i2c.do_receive(data_buf, 0, MEM_SIZE)
        elif state == rp2040_i2c_slave.POLL_RESPOND:
            # Master has initiated a read; produce the stored data back,
            # (extending with '-1' if more data is requested than
            # previously written)
            s_i2c.do_respond(iter(data_buf[0:MEM_SIZE]), -1)
        # Some real work could be done here when the bus is idle.
~~~


# Limitations

The implementation does _not_ support handling transmissions where a write
is followed by a read after a restart condition (such as e.g. `readfrom_mem` does
in Micropython's `I2C`). 

The priority has been a simple, clean, robust and fast `poll` method rather
than supporting such a case.
Moreover, the software might need some (potentially long) computation time before
being being able to respond to the subsequent read, hence it is more simple to require
that the master breaks the transmission in two (one write and one read) - with a
suitable delay between.


# Performance limits

The implementation has been developed and tested on the
[Raspberry Pi Pico](https://micropython.org/download/RPI_PICO) with MicroPython v1.24.1,
using one Pico as master (with standard Micropython), another as slave
(using this module), and a bus made of jumper wire with a rather weak pull-up of
270 Ohms.

With this setup - and the minimal (i.e., fastest realistic) slave implementation
above, operation is robust with transfers up to 256 bytes in either
direction, both with 100 kHz and 400 kHz bus speed.

If a 256-byte write on 400 kHz is immediately followed by a read, the slave is _still_
not able to keep up with the master however, because overhead when buffering the write will cause `do_receive` to lag the bus transfer by some 500 us.
If the slave needs to do any computation on the bytes written, the minimum delay
between writes and reads will need to be increased accordingly.


# Performance numbers

On the standard Raspberry Pi Pico, a single execution of `poll` takes some 37 us
with minimal variation (the original code's `handle_event` was about 60 times slower
on average, plus garbage collection in the worst case).

Due to slave overhead and clock stretching, the transfer will not achieve full speed.
The slave can buffer a write from the master relatively fast, but producing a
response to a read is _much_ slower, since the slave needs to react to ACK/NACK
after each single byte has been transferred.

When using the described setup and transferring messages of 256 bytes, the bus
utilization (as measured by the master, using MicroPython's `I2C` API) will be
up to 95% for a write (slave read), and 72% for a read (slave response),
at 100 kHz bus speed.
At 400 kHz bus speed the corresponding numbers are around 90% and 39%. YMMV.

(The original code could not reliably handle a write of more than ca. 100 bytes, or
respond about 50 bytes beore causing a timeout on the master, at 100 kHz bus speed).


# History 

The present implementation is derived from the `RP2040_Slave.py` found
[here](https://github.com/ifurusato/rp2040-i2c-slave.git),
but differs significantly due focus on low-latency operation and a high-level API.

Git tracks the evolution history.
