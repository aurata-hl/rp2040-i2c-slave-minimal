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
