/*
  minimal test program for SPL06-001 baro on I2C
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/SPIDevice.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <utility>
#include <stdio.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define SPI_DEV

#ifdef SPI_DEV
static AP_HAL::OwnPtr<AP_HAL::Device> spi_dev;
#else
static AP_HAL::OwnPtr<AP_HAL::Device> i2c_dev;
#endif

#define PRESSURE_SENSOR     0
#define TEMPERATURE_SENSOR  1
#define HW_ADR 0x76

//Pressure measure speed (sample/sec),Background mode
#define  PM_RATE_1          (0<<4)      //1 measurements pr. sec.
#define  PM_RATE_2          (1<<4)      //2 measurements pr. sec.
#define  PM_RATE_4          (2<<4)      //4 measurements pr. sec.
#define  PM_RATE_8          (3<<4)      //8 measurements pr. sec.
#define  PM_RATE_16         (4<<4)      //16 measurements pr. sec.
#define  PM_RATE_32         (5<<4)      //32 measurements pr. sec.
#define  PM_RATE_64         (6<<4)      //64 measurements pr. sec.
#define  PM_RATE_128        (7<<4)      //128 measurements pr. sec.

//Pressure sample speed (times),Background mode
#define PM_PRC_1            0       //Sigle         kP=524288   ,3.6ms
#define PM_PRC_2            1       //2 times       kP=1572864  ,5.2ms
#define PM_PRC_4            2       //4 times       kP=3670016  ,8.4ms
#define PM_PRC_8            3       //8 times       kP=7864320  ,14.8ms
#define PM_PRC_16           4       //16 times      kP=253952   ,27.6ms
#define PM_PRC_32           5       //32 times      kP=516096   ,53.2ms
#define PM_PRC_64           6       //64 times      kP=1040384  ,104.4ms
#define PM_PRC_128          7       //128 times     kP=2088960  ,206.8ms

//Temperature measure speed (sample/sec),Background mode
#define  TMP_RATE_1         (0<<4)      //1 measurements pr. sec.
#define  TMP_RATE_2         (1<<4)      //2 measurements pr. sec.
#define  TMP_RATE_4         (2<<4)      //4 measurements pr. sec.
#define  TMP_RATE_8         (3<<4)      //8 measurements pr. sec.
#define  TMP_RATE_16        (4<<4)      //16 measurements pr. sec.
#define  TMP_RATE_32        (5<<4)      //32 measurements pr. sec.
#define  TMP_RATE_64        (6<<4)      //64 measurements pr. sec.
#define  TMP_RATE_128       (7<<4)      //128 measurements pr. sec.

//Temperature sample speed(times),Background mode
#define TMP_PRC_1           0       //Sigle
#define TMP_PRC_2           1       //2 times
#define TMP_PRC_4           2       //4 times
#define TMP_PRC_8           3       //8 times
#define TMP_PRC_16          4       //16 times
#define TMP_PRC_32          5       //32 times
#define TMP_PRC_64          6       //64 times
#define TMP_PRC_128         7       //128 times

//SPL06_MEAS_CFG
#define MEAS_COEF_RDY       0x80
#define MEAS_SENSOR_RDY     0x40        // sensor initialize ready
#define MEAS_TMP_RDY        0x20        // get new temperature
#define MEAS_PRS_RDY        0x10        // get new pressure

#define MEAS_CTRL_Standby               0x00    // Standby Mode - Idle / Stop background measurement
#define MEAS_CTRL_PressMeasure          0x01    // Command Mode - Pressure measurement
#define MEAS_CTRL_TempMeasure           0x02    // Command Mode - Temperature measurement
#define MEAS_CTRL_ContinuousPress       0x05    // Background Mode - Continuous pressure measurement
#define MEAS_CTRL_ContinuousTemp        0x06    // Background Mode - Continuous temperature measurement
#define MEAS_CTRL_ContinuousPressTemp   0x07    // Background Mode - Continuous pressure and temperature measurement

//FIFO_STS
#define SPL06_FIFO_FULL     0x02
#define SPL06_FIFO_EMPTY    0x01

//INT_STS
#define SPL06_INT_FIFO_FULL     0x04
#define SPL06_INT_TMP           0x02
#define SPL06_INT_PRS           0x01

//CFG_REG
#define SPL06_CFG_T_SHIFT   0x08    //oversampling, must use it when times>8
#define SPL06_CFG_P_SHIFT   0x04

#define SP06_PSR_B2     0x00        // Pressure
#define SP06_PSR_B1     0x01
#define SP06_PSR_B0     0x02
#define SP06_TMP_B2     0x03        // Temperature
#define SP06_TMP_B1     0x04
#define SP06_TMP_B0     0x05

#define SP06_PSR_CFG    0x06        // Pressure measurement configure
#define SP06_TMP_CFG    0x07        // Temperature measurement configure
#define SP06_MEAS_CFG   0x08        // Mode configure

#define SP06_CFG_REG    0x09
#define SP06_INT_STS    0x0A
#define SP06_FIFO_STS   0x0B

#define SP06_RESET      0x0C
#define SP06_ID         0x0D

#define SP06_COEF       0x10        //-0x21
#define SP06_COEF_SRCE  0x28

struct spl0601_calib_param_t {
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
};

typedef struct spl0601_t {
    struct spl0601_calib_param_t calib_param;/**<calibration data*/
    int32_t i32rawPressure;
    int32_t i32rawTemperature;
    int32_t i32kP;
    int32_t i32kT;
} spl0601_t;

#ifdef SPI_DEV
spl0601_t spl06_spi;
#else
spl0601_t spl06_i2c;
#endif

//static float _kT,_kP;
//static float _Temp,_Press;

void setup(void);
void loop(void);

bool read_registers(uint8_t first_reg, uint8_t *recv, uint32_t recv_len);
bool block_read(uint8_t reg, uint8_t *buf, uint32_t size);
static void spl0601_write(spl0601_t *hdev, uint8_t hwadr, uint8_t regadr, uint8_t val);
static uint8_t spl0601_read(spl0601_t *hdev, uint8_t hwadr, uint8_t regadr);
void spl0601_start_continuous(spl0601_t *hdev, uint8_t mode);
static void spl0601_get_calib_param(spl0601_t *hdev);
void spl0601_rateset(spl0601_t *hdev, uint8_t iSensor, uint8_t u8SmplRate, uint8_t u8OverSmpl);
void spl0601_get_raw_pressure(spl0601_t *hdev);
void spl0601_get_raw_temp(spl0601_t *hdev);
float spl0601_get_pressure(spl0601_t *hdev);
float spl0601_get_temperature(spl0601_t *hdev);


bool read_registers(uint8_t first_reg, uint8_t *recv, uint32_t recv_len)
{
#ifdef SPI_DEV
	first_reg |= 0x80;
	return spi_dev->transfer(&first_reg, 1, recv, recv_len);
#else
    return i2c_dev->transfer(&first_reg, 1, recv, recv_len);
#endif
}

bool block_read(uint8_t reg, uint8_t *buf, uint32_t size)
{
    return read_registers(reg, buf, size);
}

static void spl0601_write(spl0601_t *hdev, uint8_t hwadr, uint8_t regadr, uint8_t val)
{
    uint8_t buf[2] = { regadr, val };
#ifdef SPI_DEV
    if (!spi_dev->transfer(buf, 2, nullptr, 0)){
    	printf("SPL06-001: failed to write\n");
    }
#else
    if (!i2c_dev->transfer(buf, 2, nullptr, 0)){
    	printf("SPL06-001: failed to write\n");
    }
#endif
}

static uint8_t spl0601_read(spl0601_t *hdev, uint8_t hwadr, uint8_t regadr)
{
    uint8_t d;
#ifdef SPI_DEV
    regadr |= 0x80;
    if (!spi_dev->transfer(&regadr, 1, &d, sizeof(d))) {
        printf("SPL06-001: failed to read\n");
    }
#else
    if (!i2c_dev->transfer(&regadr, 1, &d, sizeof(d))) {
        printf("SPL06-001: failed to read\n");
    }
#endif
    return d;
}

void spl0601_start_continuous(spl0601_t *hdev, uint8_t mode)
{
#ifdef SPI_DEV
	spl0601_write(&spl06_spi, HW_ADR, 0x08, mode);
#else
    spl0601_write(&spl06_i2c, HW_ADR, 0x08, mode);
#endif
}

static void spl0601_get_calib_param(spl0601_t *hdev)
{
    uint8_t h;
    uint8_t m;
    uint8_t l;

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

#ifdef SPI_DEV
    _dev = std::move(hal.spi->get_device("spl0601"));
#else
    _dev = std::move(hal.i2c_mgr->get_device(1, HW_ADR));
#endif

    read_registers(0x10, &h, 1);
    read_registers(0x11, &l, 1);
    hdev->calib_param.c0 = (int16_t)h<<4 | (int16_t)l>>4;
    hdev->calib_param.c0 = (hdev->calib_param.c0&0x0800)?(0xF000|hdev->calib_param.c0):hdev->calib_param.c0;
    read_registers(0x11, &h, 1);
    read_registers(0x12, &l, 1);
    hdev->calib_param.c1 = (int16_t)(h&0x0F)<<8 | (int16_t)l;
    hdev->calib_param.c1 = (hdev->calib_param.c1&0x0800)?(0xF000|hdev->calib_param.c1):hdev->calib_param.c1;
    read_registers(0x13, &h, 1);
    read_registers(0x14, &m, 1);
    read_registers(0x15, &l, 1);
    hdev->calib_param.c00 = (int32_t)h<<12 | (int32_t)m<<4 | (int32_t)l>>4;
    hdev->calib_param.c00 = (hdev->calib_param.c00&0x080000)?(0xFFF00000|hdev->calib_param.c00):hdev->calib_param.c00;
    read_registers(0x15, &h, 1);
    read_registers(0x16, &m, 1);
    read_registers(0x17, &l, 1);
    hdev->calib_param.c10 = (int32_t)(h & 0x0F) << 16 | (int32_t)m << 8 | (int32_t)l;
    hdev->calib_param.c10 = (hdev->calib_param.c10&0x080000)?(0xFFF00000|hdev->calib_param.c10):hdev->calib_param.c10;
    read_registers(0x18, &h, 1);
    read_registers(0x19, &l, 1);
    hdev->calib_param.c01 = (int16_t)h<<8 | (int16_t)l;
    read_registers(0x1A, &h, 1);
    read_registers(0x1B, &l, 1);
    hdev->calib_param.c11 = (int16_t)h<<8 | (int16_t)l;
    read_registers(0x1C, &h, 1);
    read_registers(0x1D, &l, 1);
    hdev->calib_param.c20 = (int16_t)h<<8 | (int16_t)l;
    read_registers(0x1E, &h, 1);
    read_registers(0x1F, &l, 1);
    hdev->calib_param.c21 = (int16_t)h<<8 | (int16_t)l;
    read_registers(0x20, &h, 1);
    read_registers(0x21, &l, 1);
    hdev->calib_param.c30 = (int16_t)h<<8 | (int16_t)l;
}

void spl0601_rateset(spl0601_t *hdev, uint8_t iSensor, uint8_t u8SmplRate, uint8_t u8OverSmpl)
{
    uint8_t reg = 0;
    int32_t i32kPkT = 0;
    switch(u8SmplRate)
    {
        case 2:
            reg |= (1<<4);
            break;
        case 4:
            reg |= (2<<4);
            break;
        case 8:
            reg |= (3<<4);
            break;
        case 16:
            reg |= (4<<4);
            break;
        case 32:
            reg |= (5<<4);
            break;
        case 64:
            reg |= (6<<4);
            break;
        case 128:
            reg |= (7<<4);
            break;
        case 1:
        default:
            break;
    }
    switch(u8OverSmpl)
    {
        case 2:
            reg |= 1;
            i32kPkT = 1572864;
            break;
        case 4:
            reg |= 2;
            i32kPkT = 3670016;
            break;
        case 8:
            reg |= 3;
            i32kPkT = 7864320;
            break;
        case 16:
            i32kPkT = 253952;
            reg |= 4;
            break;
        case 32:
            i32kPkT = 516096;
            reg |= 5;
            break;
        case 64:
            i32kPkT = 1040384;
            reg |= 6;
            break;
        case 128:
            i32kPkT = 2088960;
            reg |= 7;
            break;
        case 1:
        default:
            i32kPkT = 524288;
            break;
    }

    if(iSensor == PRESSURE_SENSOR)
    {
        hdev->i32kP = i32kPkT;
        spl0601_write(hdev, HW_ADR, 0x06, reg);
        if(u8OverSmpl > 8)
        {
            reg = spl0601_read(hdev, HW_ADR, 0x09);
            spl0601_write(hdev, HW_ADR, 0x09, reg | 0x04);
        }
        else
        {
            reg = spl0601_read(hdev, HW_ADR, 0x09);
            spl0601_write(hdev, HW_ADR, 0x09, reg & (~0x04));
        }
    }
    if(iSensor == TEMPERATURE_SENSOR)
    {
        hdev->i32kT = i32kPkT;
        spl0601_write(hdev, HW_ADR, 0x07, reg|0x80);  //Using mems temperature
        if(u8OverSmpl > 8)
        {
            reg = spl0601_read(hdev, HW_ADR, 0x09);
            spl0601_write(hdev, HW_ADR, 0x09, reg | 0x08);
        }
        else
        {
            reg = spl0601_read(hdev, HW_ADR, 0x09);
            spl0601_write(hdev, HW_ADR, 0x09, reg & (~0x08));
        }
    }

}

void spl0601_get_raw_pressure(spl0601_t *hdev)
{
    uint8_t buf[3];

    if (!block_read(SP06_PSR_B2, buf, 3)) {
        // can't talk on bus
    	printf("block_read failed\n");
    }

    hdev->i32rawPressure = (int32_t)buf[0]<<16 | (int32_t)buf[1]<<8 | (int32_t)buf[2];
    hdev->i32rawPressure = (hdev->i32rawPressure&0x800000) ? (0xFF000000|hdev->i32rawPressure) : hdev->i32rawPressure;
}

void spl0601_get_raw_temp(spl0601_t *hdev)
{
    uint8_t buf[3];

    if (!block_read(SP06_TMP_B2, buf, 3)) {
        // can't talk on bus
    	printf("block_read failed\n");
    }

    hdev->i32rawTemperature = (int32_t)buf[0]<<16 | (int32_t)buf[1]<<8 | (int32_t)buf[2];
    hdev->i32rawTemperature = (hdev->i32rawTemperature&0x800000) ? (0xFF000000|hdev->i32rawTemperature) : hdev->i32rawTemperature;
}

float spl0601_get_pressure(spl0601_t *hdev)
{
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;

    fTsc = hdev->i32rawTemperature / (float)hdev->i32kT;
    fPsc = hdev->i32rawPressure / (float)hdev->i32kP;
    qua2 = hdev->calib_param.c10 + fPsc * (hdev->calib_param.c20 + fPsc* hdev->calib_param.c30);
    qua3 = fTsc * fPsc * (hdev->calib_param.c11 + fPsc * hdev->calib_param.c21);

    fPCompensate = hdev->calib_param.c00 + fPsc * qua2 + fTsc * hdev->calib_param.c01 + qua3;
    return fPCompensate;
}

float spl0601_get_temperature(spl0601_t *hdev)
{
    float fTCompensate;
    float fTsc;

    fTsc = hdev->i32rawTemperature / (float)hdev->i32kT;
    fTCompensate =  hdev->calib_param.c0 * 0.5 + hdev->calib_param.c1 * fTsc;
    return fTCompensate;
}

static void dev_init(void)
{
#ifdef SPI_DEV
    if (!spi_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        printf("Failed to get baro semaphore");
    }
#else
    if (!i2c_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        AP_HAL::panic("Failed to get baro semaphore");
    }
#endif

#ifdef SPI_DEV

    spl0601_get_calib_param(&spl06_spi);
    spl0601_rateset(&spl06_spi, PRESSURE_SENSOR, 64, 4);
    spl0601_rateset(&spl06_spi, TEMPERATURE_SENSOR, 64, 4);
    spl0601_start_continuous(&spl06_spi, MEAS_CTRL_ContinuousPressTemp);

    spi_dev->get_semaphore()->give();
#else
    spl0601_get_calib_param(&spl06_i2c);
    spl0601_rateset(&spl06_i2c, PRESSURE_SENSOR, 64, 4);
    spl0601_rateset(&spl06_i2c, TEMPERATURE_SENSOR, 64, 4);
    spl0601_start_continuous(&spl06_i2c, MEAS_CTRL_ContinuousPressTemp);

    i2c_dev->get_semaphore()->give();
#endif

}

static void dev_update(void)
{
#ifdef SPI_DEV
    if (!spi_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        AP_HAL::panic("Failed to get baro semaphore");
    }
#else
    if (!i2c_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        AP_HAL::panic("Failed to get baro semaphore");
    }
#endif
    uint8_t v[14];
    float temp, press;

    memset(v, 0, sizeof(v));

#if 0
    if (!block_read(0x00, v, 14)) {
        // can't talk on bus
    	printf("block_read failed\n");
    }
#else
    for(uint8_t i = 0; i < 14; i++){
#ifdef SPI_DEV
    	v[i] = spl0601_read(&spl06_spi, HW_ADR, i);
#else
    	v[i] = spl0601_read(&spl06_i2c, HW_ADR, i);
#endif
    }
#endif

    for(uint8_t i = 0; i < 14; i++){
    	printf("0x%02X: %02X\n", i, v[i]);
    }

#ifdef SPI_DEV
    spl0601_get_raw_temp(&spl06_spi);
    spl0601_get_raw_pressure(&spl06_spi);
    temp = spl0601_get_temperature(&spl06_spi);
    press = spl0601_get_pressure(&spl06_spi);

    hal.scheduler->delay(1);

    spi_dev->get_semaphore()->give();
#else
    spl0601_get_raw_temp(&spl06_i2c);
    spl0601_get_raw_pressure(&spl06_i2c);
    temp = spl0601_get_temperature(&spl06_i2c);
    press = spl0601_get_pressure(&spl06_i2c);

    hal.scheduler->delay(1);

    i2c_dev->get_semaphore()->give();
#endif

#ifdef SPI_DEV
    printf("raw_temp: %ld\n", spl06_spi.i32rawTemperature);
    printf("raw_press: %ld\n", spl06_spi.i32rawPressure);
#else
    printf("raw_temp: %ld\n", spl06_i2c.i32rawTemperature);
    printf("raw_press: %ld\n", spl06_i2c.i32rawPressure);
#endif
    printf("temp: %f\n", temp);
    printf("press: %f\n", press);

    printf("\n");
}

void setup()
{
    AP_BoardConfig{}.init();

    hal.scheduler->delay(1000);
#ifdef SPI_DEV
    spi_dev = std::move(hal.spi->get_device("spl0601"));
#else
    i2c_dev = std::move(hal.i2c_mgr->get_device(1, HW_ADR));
#endif

    dev_init();

    while (true) {
    	dev_update();
        hal.scheduler->delay(1000);
    }
}


void loop()
{
}

AP_HAL_MAIN();
