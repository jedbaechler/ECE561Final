#include <iostream>
#include <cstdint>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
// #include "BMP280.h"
// #include "bmp2.h"
#include <linux/i2c.h>
#include <string.h>
#include "bmp280driver.h"
#include <math.h>


const char *i2c_fname = "/dev/i2c-1";
int i2c_fd = -1;

int i2c_init(void) {
    if ((i2c_fd = open(i2c_fname, O_RDWR)) < 0) {
        char err[200];
        sprintf(err, "open('%s') in i2c_init", i2c_fname);
        perror(err);
        return -1;
    }

    // NOTE we do not call ioctl with I2C_SLAVE here because we always use the I2C_RDWR ioctl operation to do
    // writes, reads, and combined write-reads. I2C_SLAVE would be used to set the I2C slave address to communicate
    // with. With I2C_RDWR operation, you specify the slave address every time. There is no need to use normal write()
    // or read() syscalls with an I2C device which does not support SMBUS protocol. I2C_RDWR is much better especially
    // for reading device registers which requires a write first before reading the response.

    return i2c_fd;
};

int i2c_write(__u8 slave_addr, __u8 reg, __u8 data) {
    int retval;
    __u8 outbuf[2];

    struct i2c_msg msgs[1];
    struct i2c_rdwr_ioctl_data msgset[1];

    outbuf[0] = reg;
    outbuf[1] = data;

    msgs[0].addr = slave_addr;
    msgs[0].flags = 0;
    msgs[0].len = 2;
    msgs[0].buf = outbuf;

    msgset[0].msgs = msgs;
    msgset[0].nmsgs = 1;

    if (ioctl(i2c_fd, I2C_RDWR, &msgset) < 0) {
        perror("ioctl(I2C_RDWR) in i2c_write");
        return -1;
    }

    return 0;
};

bmp280driver::bmp280driver(void){
    printf("Creating BMP Interaction Object\n");
}

uint16_t bmp280driver::read16(__u8 slave_addr, __u8 reg_location) {
    char store[2] = {0};
    char location[1] = {reg_location};
    write(slave_addr, location, 1);
    read(slave_addr, store, 2);
    return (store[0] << 8) | store[1];
}

uint16_t bmp280driver::read16_LE(__u8 slave_addr, __u8 reg_location) {
    uint16_t temp = read16(slave_addr, reg_location);
    return (temp >> 8) | (temp << 8);
}

int16_t bmp280driver::readS16(__u8 slave_addr, __u8 reg)
{
  return (int16_t)read16(slave_addr, reg);

}

int16_t bmp280driver::readS16_LE(__u8 slave_addr, __u8 reg)
{
  return (int16_t)read16_LE(slave_addr, reg);

}

uint32_t bmp280driver::read24(__u8 slave_addr, __u8 reg_location) {
    char store[3] = {0};
    char location[1] = {reg_location};
    write(slave_addr, location, 1);
    read(slave_addr, store, 3);
    return (store[0] << 16) | (store[1] << 8) | (store[2]);
}

void bmp280driver::readCoefficients(__u8 slave_addr)
{
    _bmp280_calib.dig_T1 = read16_LE(slave_addr, BMP280_REGISTER_DIG_T1);
    printf("T1: %d\n", _bmp280_calib.dig_T1);
    _bmp280_calib.dig_T2 = readS16_LE(slave_addr, BMP280_REGISTER_DIG_T2);
    printf("T2: %d\n", _bmp280_calib.dig_T2);
    _bmp280_calib.dig_T3 = readS16_LE(slave_addr, BMP280_REGISTER_DIG_T3);
    printf("T3: %d\n", _bmp280_calib.dig_T3);

    _bmp280_calib.dig_P1 = read16_LE(slave_addr, BMP280_REGISTER_DIG_P1);
    _bmp280_calib.dig_P2 = readS16_LE(slave_addr, BMP280_REGISTER_DIG_P2);
    _bmp280_calib.dig_P3 = readS16_LE(slave_addr, BMP280_REGISTER_DIG_P3);
    _bmp280_calib.dig_P4 = readS16_LE(slave_addr, BMP280_REGISTER_DIG_P4);
    _bmp280_calib.dig_P5 = readS16_LE(slave_addr, BMP280_REGISTER_DIG_P5);
    _bmp280_calib.dig_P6 = readS16_LE(slave_addr, BMP280_REGISTER_DIG_P6);
    _bmp280_calib.dig_P7 = readS16_LE(slave_addr, BMP280_REGISTER_DIG_P7);
    _bmp280_calib.dig_P8 = readS16_LE(slave_addr, BMP280_REGISTER_DIG_P8);
    _bmp280_calib.dig_P9 = readS16_LE(slave_addr, BMP280_REGISTER_DIG_P9);
};

float bmp280driver::readTemperature(__u8 slave_reg)
{
    int32_t var1, var2;

    int32_t adc_T = read24(slave_reg, BMP280_REGISTER_TEMPDATA);
    adc_T >>= 4;
    //   var1 = ((double) adc_T)/16384.0 - ((double) _bmp280_calib.dig_T1)/1024.0 * (double) _bmp280_calib.dig_T2;
    //   printf("Var1: %f", var1);
    //   return 0.1;


    var1  = ((((adc_T>>3) - ((int32_t)_bmp280_calib.dig_T1<<1))) *
        ((int32_t)_bmp280_calib.dig_T2)) >> 11;

    //   printf("var1: %d\n", var1);
    var2  = (((((adc_T>>4) - ((int32_t)_bmp280_calib.dig_T1)) *
            ((adc_T>>4) - ((int32_t)_bmp280_calib.dig_T1))) >> 12) *
        ((int32_t)_bmp280_calib.dig_T3)) >> 14;
    //   printf("var2: %d\n", var2);
    t_fine = var1 + var2;

    float T  = (t_fine * 5 + 128) >> 8;
    return T/100;

}

float bmp280driver::readPressure(__u8 fd) {
    int64_t var1, var2, p;

    // Must be done first to get the t_fine variable set up
    readTemperature(fd);

    int32_t adc_P = read24(fd, BMP280_REGISTER_PRESSUREDATA);
    adc_P >>= 4;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)_bmp280_calib.dig_P6;
    var2 = var2 + ((var1*(int64_t)_bmp280_calib.dig_P5)<<17);
    var2 = var2 + (((int64_t)_bmp280_calib.dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)_bmp280_calib.dig_P3)>>8) +
        ((var1 * (int64_t)_bmp280_calib.dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)_bmp280_calib.dig_P1)>>33;

    if (var1 == 0) {
        return 0;  // avoid exception caused by division by zero
    }
    p = 1048576 - adc_P;
    p = (((p<<31) - var2)*3125) / var1;
    var1 = (((int64_t)_bmp280_calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)_bmp280_calib.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280_calib.dig_P7)<<4);
    return (float)p/256;
}

float bmp280driver::readAltitude(__u8 fd, float seaLevelhPa) {
    float altitude;

    float pressure = readPressure(fd); // in Si units for Pascal
    pressure /= 100;

    altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));
    //Return altiude in ft
    return altitude*3.2808;
}

void bmp280driver::init_bmp(){
    if ((fd = open(I2C_DEV, O_RDWR)) < 0) {
        std::cerr << "Failed to open the I2C device." << std::endl;
        exit(1);
    }

    if (ioctl(fd, I2C_SLAVE, BMP280_ADDRESS) < 0) {
        std::cerr << "Failed to acquire bus access and/or talk to slave." << std::endl;
        exit(1);
    }

    char reg[1] = {0x00};
    write(fd, reg, 1);
    char data[6] = {0};
    if(read(fd, data, 6) != 6)
    {
        printf("Error : Input/Output error \n");
        exit(1);
    }


    i2c_fd = i2c_init();
    i2c_write(0x77, 0xF4, 0b00101111);

    printf("Reading temp calibration values into struct...\n");
    readCoefficients(fd);
}

//int main() {
//    int fd;
//    // struct bmp280_dev bmp;
//    // struct bmp280_config conf;
//    // struct bmp280_uncomp_data ucomp_data;
//    float altitude;

//    if ((fd = open(I2C_DEV, O_RDWR)) < 0) {
//        std::cerr << "Failed to open the I2C device." << std::endl;
//        exit(1);
//    }

//    if (ioctl(fd, I2C_SLAVE, BMP280_ADDRESS) < 0) {
//        std::cerr << "Failed to acquire bus access and/or talk to slave." << std::endl;
//        exit(1);
//    }

//    char reg[1] = {0x00};
//    write(fd, reg, 1);
//    char data[6] = {0};
//    if(read(fd, data, 6) != 6)
//    {
//        printf("Error : Input/Output error \n");
//        exit(1);
//    }


//    i2c_fd = i2c_init();
//    i2c_write(0x77, 0xF4, 0b00101111);

//    bmp280driver bmp_object = bmp280driver();


//    uint16_t val = bmp_object.read16(fd, 0x8C);
//    printf("Val: 0x%x\n", val);
//    val = bmp_object.readS16_LE(fd, 0x8C);
//    printf("Val LE: 0x%x\n", val);

//    printf("Reading temp calibration values into struct...\n");
//    bmp_object.readCoefficients(fd);
//    float temp_val = bmp_object.readTemperature(fd);
//    printf("Temperature Value Calculated: %f\n", temp_val);

//    float pressure_val = bmp_object.readPressure(fd);
//    printf("Pressure: %f\n", pressure_val);

//    while(1){
//    float altitude_val = bmp_object.readAltitude(fd);
//    printf("Altitude: %f\n", altitude_val);
//    usleep(50000);
//    // sleep(2);
//    // };
//    }
//    return 0;
//}

