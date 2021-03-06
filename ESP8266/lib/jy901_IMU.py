class Gyro:
    def __init__(self, iic, adr=0x50):
        self.i2c = iic
        self.addr = adr
        self.i2c.start()
        self.i2c.writeto(self.addr, bytearray([107, 0]))

    def get_acc(self):
        try:
            self.raw_acc_x = self.i2c.readfrom_mem(self.addr, 0x34, 2)
            self.raw_acc_y = self.i2c.readfrom_mem(self.addr, 0x35, 2)
            self.raw_acc_z = self.i2c.readfrom_mem(self.addr, 0x36, 2)
        except:
            print("ReadError: gyro_acc")
            return 0, 0, 0
        else:
            self.k_acc = 16 * 9.8
            self.acc_x = (self.raw_acc_x[1] << 8 | self.raw_acc_x[0]) / 32768 * self.k_acc
            self.acc_y = (self.raw_acc_y[1] << 8 | self.raw_acc_y[0]) / 32768 * self.k_acc
            self.acc_z = (self.raw_acc_z[1] << 8 | self.raw_acc_z[0]) / 32768 * self.k_acc
            if self.acc_x >= self.k_acc:
                self.acc_x -= 2 * self.k_acc

            if self.acc_y >= self.k_acc:
                self.acc_y -= 2 * self.k_acc

            if self.acc_z >= self.k_acc:
                self.acc_z -= 2 * self.k_acc
            self.i2c.stop()
            return self.acc_x, self.acc_y, self.acc_z

    def get_gyro(self):
        try:
            self.raw_gyro_x = self.i2c.readfrom_mem(self.addr, 0x37, 2)
            self.raw_gyro_y = self.i2c.readfrom_mem(self.addr, 0x38, 2)
            self.raw_gyro_z = self.i2c.readfrom_mem(self.addr, 0x39, 2)
        except:
            print("ReadError: gyro_gyro")
            return 0, 0, 0
        else:
            self.k_gyro = 2000
            self.gyro_x = (self.raw_gyro_x[1] << 8 | self.raw_gyro_x[0]) / 32768 * self.k_gyro
            self.gyro_y = (self.raw_gyro_y[1] << 8 | self.raw_gyro_y[0]) / 32768 * self.k_gyro
            self.gyro_z = (self.raw_gyro_z[1] << 8 | self.raw_gyro_z[0]) / 32768 * self.k_gyro
            if self.gyro_x >= self.k_gyro:
                self.gyro_x -= 2 * self.k_gyro
            if self.gyro_y >= self.k_gyro:
                self.gyro_y -= 2 * self.k_gyro
            if self.gyro_z >= self.k_gyro:
                self.gyro_z -= 2 * self.k_gyro
            self.i2c.stop()
            return self.gyro_x, self.gyro_y, self.gyro_z

    def get_angle(self):
        try:
            self.raw_angle_x = self.i2c.readfrom_mem(self.addr, 0x3d, 2)
            self.raw_angle_y = self.i2c.readfrom_mem(self.addr, 0x3e, 2)
            self.raw_angle_z = self.i2c.readfrom_mem(self.addr, 0x3f, 2)
        except:
            print("ReadError: gyro_angle")
            return 0, 0, 0
        else:
            self.k_angle = 180
            self.angle_x = (self.raw_angle_x[1] << 8 | self.raw_angle_x[0]) / 32768 * self.k_angle
            self.angle_y = (self.raw_angle_y[1] << 8 | self.raw_angle_y[0]) / 32768 * self.k_angle
            self.angle_z = (self.raw_angle_z[1] << 8 | self.raw_angle_z[0]) / 32768 * self.k_angle
            self.i2c.stop()
            return self.angle_x, self.angle_y, self.angle_z


