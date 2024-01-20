#include "LSM6DSV16B.h"

#define writeBits(dev, reg, data) i2c_reg_write_byte_dt(dev, reg, data)
#define changeBitsMask(dev, reg, mask, data) i2c_reg_update_byte_dt(dev, reg, mask, data)
#define changeBits(dev, reg, data) changeBitsMask(dev, reg, data, data)

bool setupLSM(const struct i2c_dt_spec *dev)
{
    if (readByte(dev, WHOAMI) != LSMID)
    {
        return false;
    }

    resetLSM(dev);

    changeBits(dev, ACCELRATE, ACCELRATE_DATA),
        changeBits(dev, GYRORATE, GYRORATE_DATA);
    writeBits(dev, ACGYFIFO, ACGYFIFO_DATA);
    changeBits(dev, ACCELSCALE, ACCELSCALE_DATA);

    setEmbed(dev, true);
    writeBits(dev, EMBEDFUNCA, EMBEDFUNCA_DATA);
    writeBits(dev, EMBEDFIFOA, EMBEDFIFOA_DATA);
    writeBits(dev, EMBEDINITA, EMBEDINITA_DATA);
    setEmbed(dev, false);

    changeBits(dev, LSMIFCONF, LSMINTSETUP);

    return true;
}

uint16_t getFIFONum(const struct i2c_dt_spec *dev)
{
    return (((readByte(dev, 0x1C) & 0x01) << 8) | readByte(dev, 0x1B));
}

uint8_t readByte(const struct i2c_dt_spec *dev, uint8_t reg)
{
    uint8_t read;
    i2c_reg_read_byte_dt(dev, reg, &read);
    return read;
}

void readBytes(const struct i2c_dt_spec *dev, uint8_t reg, uint8_t size, uint8_t *data)
{
    for (int i = 0; i < size; i++)
    {
        data[i] = readByte(dev, reg + i);
    }
}

void setEmbed(const struct i2c_dt_spec *dev, bool set)
{
    k_msleep(10);
    if (set)
        writeBits(dev, 0x01, 0b10000000);
    else
        writeBits(dev, 0x01, 0b00000000);
    k_msleep(10);
}

void enableWOM(const struct i2c_dt_spec *dev)
{
    resetLSM(dev);
    changeBits(dev, LSMIFCONF, LSMINTSETUP);
    writeBits(dev, GYRORATE, 0b01000000);
    writeBits(dev, ACCELRATE, 0b01100000);
    writeBits(dev, WAKESRCCONF, WAKESRCCONF_DATA);
    writeBits(dev, WAKEUPTHS, WAKEUPTHS_DATA);
    writeBits(dev, MD1CFG, MD1CFG_DATA);
    changeBitsMask(dev, FUNCTENINT, 0b10000000, FUNCTENINT_DATA);
}

void disableWOM(const struct i2c_dt_spec *dev)
{
    resetLSM(dev);
    setupLSM(dev);
}

void resetLSM(const struct i2c_dt_spec *dev)
{
    writeBits(dev, LSMCONTROL, LSMRESET);
    writeBits(dev, LSMCTRL3, LSMCLEAR);

    k_msleep(5);
}

bool checkLSMID(const struct i2c_dt_spec *dev)
{
    return readByte(dev, WHOAMI) == LSMID;
}

uint32_t previousDataTime, currentDataTime;
// float rawAcceleration[3], linacceleration[3];
// FusionQuaternion fused;
FusionQuaternion rotation = {.element = {0, 0, 1, ROTATION}};

typedef union
{
    int16_t int16[3];
    uint8_t uint8[6];
} convert;

void readFIFO(const struct i2c_dt_spec *dev, float *accel, float *laccel, float *fquat)
{
    uint16_t samples = getFIFONum(dev);
    for (uint16_t i = 0; i < samples; i++)
    {
        uint8_t tag = readByte(dev, FIFOTAG) >> 3;
        printf("%#02x ", tag);
        switch (tag)
        {
        case TAGACCEL:
        {
            int32_t intAcceleration[3];
            convert data_raw;
            float accelfloat[3];
            readBytes(dev, FIFO, 6, data_raw.uint8);
            if (accel == NULL)
                break;
            accelfloat[0] = (float)data_raw.int16[0] * ACCELSENS;
            accelfloat[1] = (float)data_raw.int16[1] * ACCELSENS;
            accelfloat[2] = (float)data_raw.int16[2] * ACCELSENS;
            intAcceleration[0] = (int32_t)accelfloat[0];
            intAcceleration[1] = (int32_t)accelfloat[1];
            intAcceleration[2] = (int32_t)accelfloat[2];
            accel[0] = (intAcceleration[0] / 1000.0f);
            accel[1] = (intAcceleration[1] / 1000.0f);
            accel[2] = (intAcceleration[2] / 1000.0f);

            break;
        }
        case TAGTIME:
        {
            if (i % samples != 0)
            {
                return;
            }
            previousDataTime = currentDataTime;
            uint32_t data[2];
            readBytes(dev, FIFO, 6, (uint8_t *)data);
            currentDataTime = data[0];
            break;
        }
        case TAGGAME:
        {
            float quat[4];
            convert data_raw;
            readBytes(dev, FIFO, 6, data_raw.uint8);
            if (fquat == NULL)
                break;
            sflp2q(quat, (uint16_t *)&data_raw.int16[0]);
            FusionQuaternion temp = {.element = {quat[3], quat[0], quat[1], quat[2]}};
            temp = FusionQuaternionMultiply(temp, rotation);
            fquat[0] = temp.element.x;
            fquat[1] = temp.element.y;
            fquat[2] = temp.element.z;
            fquat[3] = temp.element.w;
            break;
        }
        case TAGGRAVITY:
        {
            float gravityVector[3];
            convert data_raw;
            readBytes(dev, FIFO, 7, data_raw.uint8);
            if (laccel == NULL || accel == NULL)
                break;
            gravityVector[0] = ((float)data_raw.int16[0]) / 1000.0f;
            gravityVector[1] = ((float)data_raw.int16[1]) / 1000.0f;
            gravityVector[2] = ((float)data_raw.int16[2]) / 1000.0f;
            laccel[0] = accel[0] - gravityVector[0] * GRAVITY;
            laccel[1] = accel[1] - gravityVector[1] * GRAVITY;
            laccel[2] = accel[2] - gravityVector[2] * GRAVITY;
            break;
        }
        default:
        {
            char data[6];
            readBytes(dev, FIFO, 6, data);
            break;
        }
        }
    }
    if (samples)
    {
        printf("\n");
    }
}

void npy_halfbits_to_floatbits(uint16_t h, uint32_t *f)
{
    uint16_t h_exp, h_sig;
    uint32_t f_sgn, f_exp, f_sig;

    h_exp = (h & 0x7c00u);
    f_sgn = ((uint32_t)h & 0x8000u) << 16;
    switch (h_exp)
    {
    case 0x0000u: /* 0 or subnormal */
        h_sig = (h & 0x03ffu);
        /* Signed zero */
        if (h_sig == 0)
        {
            *f = f_sgn;
        }
        /* Subnormal */
        h_sig <<= 1;
        while ((h_sig & 0x0400u) == 0)
        {
            h_sig <<= 1;
            h_exp++;
        }
        f_exp = ((uint32_t)(127 - 15 - h_exp)) << 23;
        f_sig = ((uint32_t)(h_sig & 0x03ffu)) << 13;
        *f = f_sgn + f_exp + f_sig;
    case 0x7c00u: /* inf or NaN */
        /* All-ones exponent and a copy of the significand */
        *f = f_sgn + 0x7f800000u + (((uint32_t)(h & 0x03ffu)) << 13);
    default: /* normalized */
        /* Just need to adjust the exponent and shift */
        *f = f_sgn + (((uint32_t)(h & 0x7fffu) + 0x1c000u) << 13);
    }
}

void npy_half_to_float(uint16_t h, float *f)
{
    union
    {
        float ret;
        uint32_t retbits;
    } conv;
    npy_halfbits_to_floatbits(h, &conv.retbits);
    *f = conv.ret;
}

void sflp2q(float quat[4], uint16_t sflp[3])
{
    float sumsq = 0;

    npy_half_to_float(sflp[0], &quat[0]);
    npy_half_to_float(sflp[1], &quat[1]);
    npy_half_to_float(sflp[2], &quat[2]);

    for (uint8_t i = 0; i < 3; i++)
    {
        sumsq += quat[i] * quat[i];
    }

    if (sumsq > 1.0f)
    {
        float n = sqrtf(sumsq);
        quat[0] /= n;
        quat[1] /= n;
        quat[2] /= n;
        sumsq = 1.0f;
    }

    quat[3] = sqrtf(1.0f - sumsq);
}