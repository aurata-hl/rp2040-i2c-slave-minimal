# SPDX-License-Identifier: MIT

from machine import mem32
from micropython import const


# RP2040 I2C register base addresses
_I2C0_BASE = const(0x40044000)
_I2C1_BASE = const(0x40048000)
_IO_BANK0_BASE = const(0x40014000)

# RP2040 I2C register relative addresses
_I2C_IC_CLR_RD_REQ = const(0x00000050)
_I2C_IC_CLR_RESTART_DET = const(0x000000a8)
_I2C_IC_CLR_RX_DONE = const(0x00000058)
_I2C_IC_CLR_START_DET = const(0x00000064)
_I2C_IC_CLR_STOP_DET = const(0x00000060)
_I2C_IC_CLR_TX_ABRT = const(0x00000054)
_I2C_IC_CON = const(0x00000000)
_I2C_IC_DATA_CMD = const(0x00000010)
_I2C_IC_ENABLE = const(0x0000006c)
_I2C_IC_INTR_STAT = const(0x0000002c)
_I2C_IC_RAW_INTR_STAT = const(0x00000034)
_I2C_IC_SAR = const(0x00000008)
_I2C_IC_STATUS = const(0x00000070)

# RP2040 I2C register bitmasks
_I2C_IC_CON__IC_SLAVE_DISABLE = const(0x00000040)
_I2C_IC_CON__MASTER_MODE = const(0x00000001)
_I2C_IC_CON__RX_FIFO_FULL_HLD_CTRL = const(0x00000200)
_I2C_IC_CON__STOP_DET_IFADDRESSED = const(0x00000080)
_I2C_IC_DATA_CMD__DAT = const(0x000000ff)
_I2C_IC_DATA_CMD__FIRST_DATA_BYTE = const(0x00000800)
_I2C_IC_ENABLE__ENABLE = const(0x00000001)
_I2C_IC_INTR_STAT__R_RD_REQ = const(0x00000020)
_I2C_IC_INTR_STAT__R_RESTART_DET = const(0x00001000)
_I2C_IC_INTR_STAT__R_RX_DONE = const(0x00000080)
_I2C_IC_INTR_STAT__R_START_DET = const(0x00000400)
_I2C_IC_INTR_STAT__R_STOP_DET = const(0x00000200)
_I2C_IC_INTR_STAT__R_TX_ABRT = const(0x00000040)
_I2C_IC_RAW_INTR_STAT__RD_REQ = const(0x00000020)
_I2C_IC_SAR__IC_SAR = const(0x000003ff)
_I2C_IC_STATUS__RFNE = const(0x00000008)


# Atomic Register Access
_MEM_RW = const(0x0000)  # Normal read/write access
_MEM_XOR = const(0x1000)  # XOR on write
_MEM_SET = const(0x2000)  # Bitmask set on write
_MEM_CLR = const(0x3000)  # Bitmask clear on write


# The register read/write functions should be simplified as much as possible,
# since these functions are invoked most frequently, and hence contribute
# a considerable amount of overhead.
# We save 1 argument and convert 1 attribute access to a compile-time
# constant by referring these callables rather than using a common
# member method.
#
# _reg_write_xxx: Write, set or clear RP2040 I2C 32bits register `register`.
# The "atomic read", `atr`, should be one of the predefined constants,
# i.e., `_MEM_RW`, `_MEM_SET`, `_MEM_CLR` or `_MEM_XOR`.
#
# _reg_read_xxx: Read the contents of the 32-bit register at the
# specified `offset`.

def _reg_write_i2c0(register, data, atr=_MEM_RW):
    mem32[_I2C0_BASE | atr | register] = data


def _reg_write_i2c1(register, data, atr=_MEM_RW):
    mem32[_I2C1_BASE | atr | register] = data


def _reg_read_i2c0(offset):
    return mem32[_I2C0_BASE | offset]


def _reg_read_i2c1(offset):
    return mem32[_I2C1_BASE | offset]


class i2c_slave:

    def __init__(self, i2cID=0, sda=0, scl=1, slaveAddress=0x44):
        self.scl = scl
        self.sda = sda
        self.slaveAddress = slaveAddress

        # Use pre-defined callables to reduce the number of arguments passed
        # per invocation to the register read and write functions
        self.i2c_ID = i2cID
        if self.i2c_ID == 0:
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
            _I2C_IC_SAR, self.slaveAddress & _I2C_IC_SAR__IC_SAR, _MEM_SET)

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

        print(
            ('established I2C slave on ID={}; SDA={}; SCL={} ' +
             'at 0x{:02X}').format(i2cID, sda, scl, self.slaveAddress))

    class I2CStateMachine:
        I2C_RECEIVE = 0
        I2C_REQUEST = 1
        I2C_FINISH = 2
        I2C_START = 3

    class I2CTransaction:
        def __init__(self, buffer, buffer_size: int):
            if buffer is None:
                self.buffer = bytearray(buffer_size)
            else:
                self.buffer = buffer
            self.buffer_size = buffer_size
            self.size = 0

        def append(self, b: int):
            if self.size >= self.buffer_size:
                return False
            self.buffer[self.size] = b
            self.size += 1

        def reset(self):
            self.size = 0

    def handle_event(self):
        # Read the I2C_IC_INTR_STAT register just once.
        intr_stat = self._reg_read(_I2C_IC_INTR_STAT)

        # I2C Master has abort the transactions
        if (intr_stat & _I2C_IC_INTR_STAT__R_TX_ABRT):
            # Clear int
            self._reg_read(_I2C_IC_CLR_TX_ABRT)
            return i2c_slave.I2CStateMachine.I2C_FINISH

        # Last byte transmitted by I2C Slave but NACK from I2C Master
        if (intr_stat & _I2C_IC_INTR_STAT__R_RX_DONE):
            # Clear int
            self._reg_read(_I2C_IC_CLR_RX_DONE)
            return i2c_slave.I2CStateMachine.I2C_FINISH

        # Restart condition detected
        if (intr_stat & _I2C_IC_INTR_STAT__R_RESTART_DET):
            # Clear int
            self._reg_read(_I2C_IC_CLR_RESTART_DET)

        # Start condition detected by I2C Slave
        if (intr_stat & _I2C_IC_INTR_STAT__R_START_DET):
            # Clear start detection
            self._reg_read(_I2C_IC_CLR_START_DET)
            return i2c_slave.I2CStateMachine.I2C_START

        # Stop condition detected by I2C Slave
        if (intr_stat & _I2C_IC_INTR_STAT__R_STOP_DET):
            # Clear stop detection
            self._reg_read(_I2C_IC_CLR_STOP_DET)
            return i2c_slave.I2CStateMachine.I2C_FINISH

        # Check if RX FIFO is not empty
        if (self._reg_read(_I2C_IC_STATUS) & _I2C_IC_STATUS__RFNE):
            return i2c_slave.I2CStateMachine.I2C_RECEIVE

        # Check if Master is requesting data
        if (intr_stat & _I2C_IC_INTR_STAT__R_RD_REQ):
            # Shall Wait until transfer is done, timing recommended
            # 10 * fastest SCL clock period:
            # for 100 Khz = (1/100E3) * 10 = 100 uS
            # for 400 Khz = (1/400E3) * 10 = 25 uS
            return i2c_slave.I2CStateMachine.I2C_REQUEST

    def is_Master_Req_Read(self):
        """ Return status if I2C Master is requesting a read sequence """
        # Check RD_REQ Interrupt bit (master wants to read data from the slave)
        v = bool(self._reg_read(_I2C_IC_RAW_INTR_STAT) &
                 _I2C_IC_RAW_INTR_STAT__RD_REQ)
        return v

    def Slave_Write_Data(self, data):
        """ Write 8 its of data at destination of I2C Master """
        self._reg_write(_I2C_IC_DATA_CMD, data & _I2C_IC_DATA_CMD__DAT)
        self._reg_read(_I2C_IC_CLR_RD_REQ)

    def Available(self):
        """ Return true if data has been received from I2C Master """

        # Get RFNE Bit (Receive FIFO Not Empty)
        return bool(self._reg_read(_I2C_IC_STATUS) & _I2C_IC_STATUS__RFNE)

    def Read_Data_Received(self):
        """ Return data from I2C Master """
        return self._reg_read(_I2C_IC_DATA_CMD) & _I2C_IC_DATA_CMD__DAT
