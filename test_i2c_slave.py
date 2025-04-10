def test_slave(addr, bus=0, sda=0, scl=1):
    """
    Example code demonstrating the use of RP2040_Slave.
    It creates a simple slave on the specified `addr` and
    RP2040 `bus`, using the GPIO pins `sda` and `scl`.
    """
    from RP2040_Slave import i2c_slave

    # Initialize an empty buffer list for sequential write sequences

    data_buf = []
    s_i2c = i2c_slave(bus, sda=sda, scl=scl, slaveAddress=addr)
    state = s_i2c.I2CStateMachine.I2C_START
    currentTransaction = s_i2c.I2CTransaction(addr, data_buf)
    counter = 0

    print(f"I2C Slave test, address {addr:02x}, bus {bus}, pins {sda}:{scl}")
    try:
        while True:
            state = s_i2c.handle_event()

            if state is None:
                continue

            if state == s_i2c.I2CStateMachine.I2C_START:
                continue

            if state == s_i2c.I2CStateMachine.I2C_RECEIVE:
                if currentTransaction.address == 0x00:
                    # First byte received is the register address
                    currentTransaction.address = s_i2c.Read_Data_Received()

                # Read all data byte received until RX FIFO is empty
                while (s_i2c.Available()):
                    currentTransaction.data_byte.append(
                        s_i2c.Read_Data_Received())
                    # Virtually Increase register address
                    # s_i2c.I2CTransaction.address += 1

            if state == s_i2c.I2CStateMachine.I2C_REQUEST:
                # Send some dummy data back
                while (s_i2c.is_Master_Req_Read()):
                    counter += 1
                    s_i2c.Slave_Write_Data(counter)

                    # Virtually Increase register address
                    # s_i2c.I2CTransaction.address += 1
                    print("Sending data : ", counter)

            if state == s_i2c.I2CStateMachine.I2C_FINISH:
                print(
                    "Register : ", currentTransaction.address,
                    "Received : ", currentTransaction.data_byte)

                currentTransaction.address = 0x00
                currentTransaction.data_byte = []

    except KeyboardInterrupt:
        pass
