#ifndef LSM6DSV16B
#define LSM6DSV16B

#include <stdio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <hal/nrf_gpio.h>
#include <zephyr/kernel.h>
#include "Fusion/FusionMath.h"

bool setupLSM(const struct i2c_dt_spec *dev);
uint16_t getFIFONum(const struct i2c_dt_spec *dev);
uint8_t readByte(const struct i2c_dt_spec *dev, uint8_t reg);
void readBytes(const struct i2c_dt_spec *dev, uint8_t reg, uint8_t size, uint8_t *data);
void setEmbed(const struct i2c_dt_spec *dev, bool set);
void enableWOM(const struct i2c_dt_spec *dev);
void disableWOM(const struct i2c_dt_spec *dev);
void resetLSM(const struct i2c_dt_spec *dev);
bool checkLSMID(const struct i2c_dt_spec *dev);
void readFIFO(const struct i2c_dt_spec *dev, float *accel, float *laccel, float *fquat);

void npy_halfbits_to_floatbits(uint16_t h, uint32_t *f);
void npy_half_to_float(uint16_t h, float *f);
void sflp2q(float quat[4], uint16_t sflp[3]);

#define LSMCONTROL 0x01
#define LSMCTRL3 0x12

#define WHOAMI 0x0F
#define LSMID 0x71

#define LSMRESET 0b00000100
#define LSMCLEAR 0b00000001
#define EMBED 0b10000000

#define ACCELRATE 0x10
#define ACCELRATE_DATA 0b00001000

#define GYRORATE 0x11
#define GYRORATE_DATA 0b00001000

#define ACGYFIFO 0x09
#define ACGYFIFO_DATA 0b00000110

#define ACCELSCALE 0x17
#define ACCELSCALE_DATA 0b00000011
#define ACCELSCALE_FILTER 0b01000000
#if ACCELSCALE_DATA == 0b00000000
#define ACCELSENS 0.061f
#elif ACCELSCALE_DATA == 0b00000001
#define ACCELSENS 0.122f
#elif ACCELSCALE_DATA == 0b00000010
#define ACCELSENS 0.244f
#elif ACCELSCALE_DATA == 0b00000011
#define ACCELSENS 0.488f
#endif

#define EMBEDFUNCA 0x04
#define EMBEDFUNCA_DATA 0b00000010

#define EMBEDFIFOA 0x44
#define EMBEDFIFOA_DATA 0b00010010

#define EMBEDINITA 0x66
#define EMBEDINITA_DATA 0b00000010

#define LSMIFCONF 0x03
#define LSMINTSETUP 0b01000100 // this is that way to set the voltage on the bq21080 to 4.18 volts. not the 4.2 volts i'd want but its ok



#define WAKESRCCONF 0x45
#define WAKESRCCONF_DATA 0b00000111

#define WAKEUPTHS 0x5B
#define WAKEUPTHS_DATA 0b00111111

#define MD1CFG 0x5E
#define MD1CFG_DATA 0b00100000

#define FUNCTENINT 0x50
#define FUNCTENINT_DATA 0b10000000

#define CTRL9 0x18
#define XFILTER 0b00001000

#define CTRL7 0x16
#define GFILTER 0b00000001

#define GYROSCALE 0x15
#define GYROSCALE_FILTER 0b00100000

#define OUTZA 0x28
#define OUTYA 0x2A
#define OUTXA 0x2C

#define FIFOTAG 0x78
#define FIFO 0x79

#define TAGACCEL 0x02
#define TAGTIME 0x04
#define TAGGAME 0x13
#define TAGGRAVITY 0x17

#define GRAVITY 9.80665f

#define ROTATION 0

#endif