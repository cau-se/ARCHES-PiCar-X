class SMBusStub:
    def __init__(self, bus):
        self.bus = bus
        self.data = {}

    def write_byte_data(self, i2c_addr, register, value):
        self.data[(i2c_addr, register)] = value

    def read_byte_data(self, i2c_addr, register):
        return self.data.get((i2c_addr, register), 0)