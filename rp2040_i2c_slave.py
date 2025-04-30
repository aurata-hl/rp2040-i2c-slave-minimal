# SPDX-License-Identifier: MIT

from machine import mem32
from micropython import const


# RP2040 I2C register base addresses
_I2C0_BASE = const(0x40044000)
_I2C1_BASE = const(0x40048000)
_IO_BANK0_BASE = const(0x40014000)

# RP2040 I2C register relative addresses
_I2C_IC_CLR_RD_REQ = const(0x00000050)
_I2C_IC_CLR_INTR = const(0x00000040)
_I2C_IC_CON = const(0x00000000)
_I2C_IC_DATA_CMD = const(0x00000010)
_I2C_IC_ENABLE = const(0x0000006c)
_I2C_IC_INTR_STAT = const(0x0000002c)
_I2C_IC_INTR_MASK = const(0x00000030)
_I2C_IC_RAW_INTR_STAT = const(0x00000034)
_I2C_IC_SAR = const(0x00000008)
_I2C_IC_STATUS = const(0x00000070)

# RP2040 I2C register bitmasks
_I2C_IC_CON__IC_SLAVE_DISABLE = const(0x00000040)
_I2C_IC_CON__MASTER_MODE = const(0x00000001)
_I2C_IC_CON__RX_FIFO_FULL_HLD_CTRL = const(0x00000200)
_I2C_IC_DATA_CMD__DAT = const(0x000000ff)
_I2C_IC_ENABLE__ENABLE = const(0x00000001)
_I2C_IC_INTR_STAT__R_RD_REQ = const(0x00000020)
_I2C_IC_INTR_STAT__R_RX_DONE = const(0x00000080)
_I2C_IC_INTR_STAT__R_START_DET = const(0x00000400)
_I2C_IC_INTR_STAT__R_STOP_DET = const(0x00000200)
_I2C_IC_INTR_STAT__R_TX_ABRT = const(0x00000040)
_I2C_IC_INTR_MASK__M_STOP_DET = const(0x00000200)
_I2C_IC_SAR__IC_SAR = const(0x000003ff)
_I2C_IC_STATUS__RFNE = const(0x00000008)

# R_TX_ABRT, R_RX_DONE or R_STOP_DET
_I2C_IC_INTR__TERMINAL = const(0x000002c0)


# Atomic Register Access
_MEM_RW = const(0x0000)  # Normal read/write access
_MEM_XOR = const(0x1000)  # XOR on write
_MEM_SET = const(0x2000)  # Bitmask set on write
_MEM_CLR = const(0x3000)  # Bitmask clear on write


# The register read/write functions should be simplified as much as possible,
# since these functions are invoked most frequently, and hence contribute
# a considerable amount of overhead.
# Hence we use two sets of more compact functions, such that the runtime
# overhead is minimized.
#
# _reg_write_xxx: Write, set or clear RP2040 I2C 32bits register `register`.
# The "atomic read", `atr`, should be one of the predefined constants,
# i.e., `_MEM_RW`, `_MEM_SET`, `_MEM_CLR` or `_MEM_XOR`.
#
# _reg_read_xxx: Read the contents of the 32-bit register at the
# specified `offset`.

def _reg_write_i2c0(register, data, atr=_MEM_RW):
    # Write register for I2C0
    mem32[_I2C0_BASE | atr | register] = data


def _reg_write_i2c1(register, data, atr=_MEM_RW):
    # Write register for I2C1
    mem32[_I2C1_BASE | atr | register] = data


def _reg_read_i2c0(offset):
    # Read register for I2C0
    return mem32[_I2C0_BASE | offset]


def _reg_read_i2c1(offset):
    # Read register for I2C1
    return mem32[_I2C1_BASE | offset]


# POLL_IDLE: The bus is idle, with nothing to be done by the slave.
POLL_IDLE = const(0x00)

# POLL_RECEIVE: The master is initiating a write operation which should be
# handled immediately.
POLL_RECEIVE = const(0x01)

# POLL_RESPOND: The master is initiating a read operation which should be
# handled immediately.
POLL_RESPOND = const(0x02)


class I2C_Slave:
    """
    An I2C slave implemented Micropython for RP2040, with a high-level
    API.

    The implementation is optimized for fast response times and minimal
    RAM usage.
    """

    def __init__(self, bus_id=0, sda=0, scl=1, address=0x44):
        """
        Constructor.
        `bus_id` is the RP204 I2C bus function (0 or 1).
        `sda` is the GPIO number of the pin used for the SDA signal.
        `scl` is the GPIO number of the pin used for SCL signal.
        `address` is the slave address.
        """
        # Expose the basic properties to the client
        self.scl = scl
        self.sda = sda
        self.address = address

        # Use pre-defined callables to simplify the (frequently executed)
        # I2C-register read and write functions
        self.bus_id = bus_id
        if self.bus_id == 0:
            self._reg_write = _reg_write_i2c0
            self._reg_read = _reg_read_i2c0
        else:
            self._reg_write = _reg_write_i2c1
            self._reg_read = _reg_read_i2c1

        """
          I2C Slave Mode Intructions
          https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf
        """

        # 1. Disable the DW_apb_i2c by writing a ‘0’ to IC_ENABLE.ENABLE
        self._reg_write(_I2C_IC_ENABLE, _I2C_IC_ENABLE__ENABLE, _MEM_CLR)

        # 2. Write to the IC_SAR register (bits 9:0) to set the slave address.
        # This is the address to which the DW_apb_i2c responds.
        self._reg_write(_I2C_IC_SAR, _I2C_IC_SAR__IC_SAR, _MEM_CLR)

        self._reg_write(
            _I2C_IC_SAR, self.address & _I2C_IC_SAR__IC_SAR, _MEM_SET)

        # 3. Write to the IC_CON register to specify which type of addressing
        # is supported (7-bit or 10-bit by setting bit 3).
        # Enable the DW_apb_i2c in slave-only mode by writing a ‘0’ into
        # bit six (IC_SLAVE_DISABLE) and a ‘0’ to bit zero
        # (MASTER_MODE).

        # Disable Master mode
        self._reg_write(_I2C_IC_CON, _I2C_IC_CON__MASTER_MODE, _MEM_CLR)

        # Enable slave mode
        self._reg_write(_I2C_IC_CON, _I2C_IC_CON__IC_SLAVE_DISABLE, _MEM_CLR)

        # Enable clock strech
        self._reg_write(
            _I2C_IC_CON, _I2C_IC_CON__RX_FIFO_FULL_HLD_CTRL, _MEM_SET)

        # Enable stop-condition detection
        self._reg_write(
            _I2C_IC_INTR_MASK, _I2C_IC_INTR_MASK__M_STOP_DET, _MEM_SET)

        # 4. Enable the DW_apb_i2c by writing a ‘1’ to IC_ENABLE.ENABLE.
        self._reg_write(_I2C_IC_ENABLE, _I2C_IC_ENABLE__ENABLE, _MEM_SET)

        # Reset GPIO0 function
        sda_base = 4 + 8 * self.sda
        mem32[_IO_BANK0_BASE | _MEM_CLR | sda_base] = 0x1f
        # Set GPIO0 as IC0_SDA function
        mem32[_IO_BANK0_BASE | _MEM_SET | sda_base] = 0x03

        # Reset GPIO1 function
        scl_base = 4 + 8 * self.scl
        mem32[_IO_BANK0_BASE | _MEM_CLR | scl_base] = 0x1f
        # Set GPIO1 as IC0_SCL function
        mem32[_IO_BANK0_BASE | _MEM_SET | scl_base] = 0x03

        # Clear all interrupts
        self._clear_interrupts()

    def _clear_interrupts(self):
        """Clear all latched I2C interrupts."""
        self._reg_read(_I2C_IC_CLR_INTR)

    def poll(self):
        """
        Poll for an I2C event: `POLL_IDLE`, `POLL_RECEIVE` or `POLL_RESPOND`.

        The most common case is `POLL_IDLE`: no activity on the bus.
        This case is optimized for low overhead.

        In case of `POLL_RECEIVE`, the master is already trying to send data,
        hence the client should immediately invoke `do_receive` to receive
        the data.

        In case of `POLL_RESPOND`, the master is already requesting data, hence
        the client should immediately invoke `do_respond` to produce the
        response.
        """
        # Read the I2C_IC_INTR_STAT register just once.
        intr_stat = self._reg_read(_I2C_IC_INTR_STAT)

        if (intr_stat & _I2C_IC_INTR__TERMINAL):
            # Just clear up and return idle if a terminal state (e.g. for a
            # prior event) was detected between polls.
            # Explicit clearing of interrupts here ensures that the next poll
            # starts from a well-defined state.
            self._clear_interrupts()
            return POLL_IDLE

        # Check whether a read-request was detected.
        # (Read is checked before write, because latency is more critical for
        # reads, and the status bit is already available).
        if (intr_stat & _I2C_IC_INTR_STAT__R_RD_REQ):
            return POLL_RESPOND

        # Check whether a write has been initiated by the master, and at least
        # one byte has been stored in the FIFO.
        if (self._reg_read(_I2C_IC_STATUS) & _I2C_IC_STATUS__RFNE):
            return POLL_RECEIVE

        # No significant event.
        return POLL_IDLE

    def do_respond(
            self,
            generator,
            fallback=0x00) -> int:
        """
        After a `POLL_RESPOND` this method should be invoked to provide the
        master with the requested data.
        Data will be taken from the `generator`. If the sequence runs out,
        the `fallback` value will be sent instead until the master stops
        requesting data.
        """
        n_bytes = 0
        in_range = True
        value = fallback

        while True:
            # Get the next value
            if in_range:
                try:
                    value = next(generator)
                except StopIteration:
                    in_range = False
                    value = fallback

            # Transmit the value, and clear the RD_REQ interrupt
            self._reg_write(_I2C_IC_DATA_CMD, value & _I2C_IC_DATA_CMD__DAT)
            self._reg_read(_I2C_IC_CLR_RD_REQ)
            n_bytes += 1

            while True:
                # Check the status
                status = self._reg_read(_I2C_IC_RAW_INTR_STAT)
                if status & _I2C_IC_INTR_STAT__R_RD_REQ:
                    # Master requests more data; continue the outer loop
                    break
                elif status & _I2C_IC_INTR__TERMINAL:
                    # Done.
                    self._clear_interrupts()
                    return n_bytes

    def do_receive(
            self,
            buffer,
            index: int,
            max_bytes=0x7FFFFFFF) -> int:
        """
        After a `POLL_RECEIVE` this method should be used to retrieve the
        data written by the master.
        Data is written into the `buffer` (via indexed assignment), starting
        at the specified `index`.
        If the master transmits more than `max_bytes` bytes of data,
        the excess data will be discarded.
        Return the number of bytes received (including discarded data).
        """
        received = 0

        # On entry there will be at least one byte in the FIFO (precondition
        # for POLL_RECEIVE), hence we start the loop with a read.
        while True:
            # Read the next byte
            next_byte = self._reg_read(
                _I2C_IC_DATA_CMD) & _I2C_IC_DATA_CMD__DAT
            received += 1

            # Store the received byte, if there's still space
            if received <= max_bytes:
                buffer[index] = next_byte
                index += 1

            n_polls = 0
            while True:
                # Check whether queued data is available
                available = self._reg_read(
                    _I2C_IC_STATUS) & _I2C_IC_STATUS__RFNE
                if available:
                    # Continue the outer loop
                    break

                # The stop condition may be detected before the last 1-2 bytes
                # have detected with _I2C_IC_STATUS__RFNE, hence we need this
                # ugly heuristic:
                n_polls += 1
                if n_polls < 3:
                    continue

                # Stop when no more data available and the transfer is finished
                status = self._reg_read(_I2C_IC_INTR_STAT)
                if (status & _I2C_IC_INTR__TERMINAL):
                    self._clear_interrupts()
                    return received
