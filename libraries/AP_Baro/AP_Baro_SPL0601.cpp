/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "AP_Baro_SPL0601.h"

#include <utility>
#include <stdio.h>

#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger2.h>

extern const AP_HAL::HAL &hal;

#define PRESSURE_SENSOR     0
#define TEMPERATURE_SENSOR  1

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

/*
  constructor
 */
AP_Baro_SPL0601::AP_Baro_SPL0601(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev)
    : AP_Baro_Backend(baro)
    , _dev(std::move(dev))
    , _lowpass_filter(20.0f)
{
}

AP_Baro_Backend *AP_Baro_SPL0601::probe(AP_Baro &baro,
                                        AP_HAL::OwnPtr<AP_HAL::Device> _dev)
{
    if (!_dev) {
        return nullptr;
    }

    AP_Baro_SPL0601 *sensor = new AP_Baro_SPL0601(baro, std::move(_dev));
    if (!sensor || !sensor->_init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_Baro_SPL0601::_init()
{
    if (!_dev) {
        return false;
    }

    if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        AP_HAL::panic("PANIC: AP_Baro_SPL0601: failed to take serial semaphore for init");
    }

    _has_sample = false;

    // SPI reads have flag 0x80 set
    _dev->set_read_flag(0x80);

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
    
    uint8_t whoami;
    if (!_dev->read_registers(SP06_ID, &whoami, 1) ||
    	whoami != 0x10) {
    	// not a SPL06-001
    	_dev->get_semaphore()->give();
    	return false;
    }
    printf("SPL0601 ID 0x%x\n", whoami);

    _spl0601_get_calib_param(&_spl0601);
    _spl0601_rateset(&_spl0601, PRESSURE_SENSOR, 64, 4);
    _spl0601_rateset(&_spl0601, TEMPERATURE_SENSOR, 64, 4);
    _spl0601_start_continuous(&_spl0601, MEAS_CTRL_ContinuousPressTemp);

    _instance = _frontend.register_sensor();
    
    _dev->get_semaphore()->give();

    /* Request 50Hz update */
    _dev->register_periodic_callback(20 * AP_USEC_PER_MSEC,
                                     FUNCTOR_BIND_MEMBER(&AP_Baro_SPL0601::_timer, void));
    return true;
}

void AP_Baro_SPL0601::_timer(void)
{
	float press_raw;

    _spl0601_get_raw_temp(&_spl0601);
    _spl0601_get_raw_pressure(&_spl0601);
    _temperature = _spl0601_get_temperature(&_spl0601);
    press_raw    = _spl0601_get_pressure(&_spl0601);

    _pressure = _lowpass_filter.apply(press_raw, 0.02f);
    AP::logger2().Log_Write_BaroLowpass(_pressure, press_raw);

    _has_sample = true;
}

bool AP_Baro_SPL0601::_block_read(uint8_t reg, uint8_t *buf, uint32_t size)
{
    return _dev->read_registers(reg, buf, size);
}

void AP_Baro_SPL0601::update()
{
    if (!_has_sample) {
        return;
    }

    WITH_SEMAPHORE(_sem);
    _copy_to_frontend(_instance, _pressure, _temperature);
    _has_sample = false;
}

void AP_Baro_SPL0601::_spl0601_write(_spl0601_t *hdev, uint8_t hwadr, uint8_t regadr, uint8_t val)
{
    uint8_t buf[2] = { regadr, val };
    if (!_dev->transfer(buf, 2, nullptr, 0)){
    	printf("SPL06-001: failed to write\n");
    }
}

uint8_t AP_Baro_SPL0601::_spl0601_read(_spl0601_t *hdev, uint8_t hwadr, uint8_t regadr)
{
    uint8_t d;

    if (!_dev->read_registers(regadr, &d, sizeof(d))) {
        printf("SPL06-001: failed to read\n");
    }
    return d;
}

void AP_Baro_SPL0601::_spl0601_start_continuous(_spl0601_t *hdev, uint8_t mode)
{
    _spl0601_write(&_spl0601, HAL_BARO_SPL06_I2C_ADDR, SP06_MEAS_CFG, mode);
}

void AP_Baro_SPL0601::_spl0601_get_calib_param(_spl0601_t *hdev)
{
    uint8_t h;
    uint8_t m;
    uint8_t l;

    _dev->read_registers(0x10, &h, 1);
    _dev->read_registers(0x11, &l, 1);
    hdev->calib_param.c0 = (int16_t)h<<4 | (int16_t)l>>4;
    hdev->calib_param.c0 = (hdev->calib_param.c0&0x0800)?(0xF000|hdev->calib_param.c0):hdev->calib_param.c0;
    _dev->read_registers(0x11, &h, 1);
    _dev->read_registers(0x12, &l, 1);
    hdev->calib_param.c1 = (int16_t)(h&0x0F)<<8 | (int16_t)l;
    hdev->calib_param.c1 = (hdev->calib_param.c1&0x0800)?(0xF000|hdev->calib_param.c1):hdev->calib_param.c1;
    _dev->read_registers(0x13, &h, 1);
    _dev->read_registers(0x14, &m, 1);
    _dev->read_registers(0x15, &l, 1);
    hdev->calib_param.c00 = (int32_t)h<<12 | (int32_t)m<<4 | (int32_t)l>>4;
    hdev->calib_param.c00 = (hdev->calib_param.c00&0x080000)?(0xFFF00000|hdev->calib_param.c00):hdev->calib_param.c00;
    _dev->read_registers(0x15, &h, 1);
    _dev->read_registers(0x16, &m, 1);
    _dev->read_registers(0x17, &l, 1);
    hdev->calib_param.c10 = (int32_t)(h & 0x0F) << 16 | (int32_t)m << 8 | (int32_t)l;
    hdev->calib_param.c10 = (hdev->calib_param.c10&0x080000)?(0xFFF00000|hdev->calib_param.c10):hdev->calib_param.c10;
    _dev->read_registers(0x18, &h, 1);
    _dev->read_registers(0x19, &l, 1);
    hdev->calib_param.c01 = (int16_t)h<<8 | (int16_t)l;
    _dev->read_registers(0x1A, &h, 1);
    _dev->read_registers(0x1B, &l, 1);
    hdev->calib_param.c11 = (int16_t)h<<8 | (int16_t)l;
    _dev->read_registers(0x1C, &h, 1);
    _dev->read_registers(0x1D, &l, 1);
    hdev->calib_param.c20 = (int16_t)h<<8 | (int16_t)l;
    _dev->read_registers(0x1E, &h, 1);
    _dev->read_registers(0x1F, &l, 1);
    hdev->calib_param.c21 = (int16_t)h<<8 | (int16_t)l;
    _dev->read_registers(0x20, &h, 1);
    _dev->read_registers(0x21, &l, 1);
    hdev->calib_param.c30 = (int16_t)h<<8 | (int16_t)l;
}

void AP_Baro_SPL0601::_spl0601_rateset(_spl0601_t *hdev, uint8_t iSensor, uint8_t u8SmplRate, uint8_t u8OverSmpl)
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
        _spl0601_write(hdev, HAL_BARO_SPL06_I2C_ADDR, SP06_PSR_CFG, reg);
        if(u8OverSmpl > 8)
        {
            reg = _spl0601_read(hdev, HAL_BARO_SPL06_I2C_ADDR, SP06_CFG_REG);
            _spl0601_write(hdev, HAL_BARO_SPL06_I2C_ADDR, SP06_CFG_REG, reg | 0x04);
        }
        else
        {
            reg = _spl0601_read(hdev, HAL_BARO_SPL06_I2C_ADDR, SP06_CFG_REG);
            _spl0601_write(hdev, HAL_BARO_SPL06_I2C_ADDR, SP06_CFG_REG, reg & (~0x04));
        }
    }
    if(iSensor == TEMPERATURE_SENSOR)
    {
        hdev->i32kT = i32kPkT;
        _spl0601_write(hdev, HAL_BARO_SPL06_I2C_ADDR, SP06_TMP_CFG, reg|0x80);  //Using mems temperature
        if(u8OverSmpl > 8)
        {
            reg = _spl0601_read(hdev, HAL_BARO_SPL06_I2C_ADDR, SP06_CFG_REG);
            _spl0601_write(hdev, HAL_BARO_SPL06_I2C_ADDR, SP06_CFG_REG, reg | 0x08);
        }
        else
        {
            reg = _spl0601_read(hdev, HAL_BARO_SPL06_I2C_ADDR, SP06_CFG_REG);
            _spl0601_write(hdev, HAL_BARO_SPL06_I2C_ADDR, SP06_CFG_REG, reg & (~0x08));
        }
    }

}

void AP_Baro_SPL0601::_spl0601_get_raw_pressure(_spl0601_t *hdev)
{
    uint8_t buf[3];

    if (!_block_read(SP06_PSR_B2, buf, 3)) {
        // can't talk on bus
    	printf("block_read failed\n");
    }

    hdev->i32rawPressure = (int32_t)buf[0]<<16 | (int32_t)buf[1]<<8 | (int32_t)buf[2];
    hdev->i32rawPressure= (hdev->i32rawPressure&0x800000) ? (0xFF000000|hdev->i32rawPressure) : hdev->i32rawPressure;
}

void AP_Baro_SPL0601::_spl0601_get_raw_temp(_spl0601_t *hdev)
{
    uint8_t buf[3];

    if (!_block_read(SP06_TMP_B2, buf, 3)) {
        // can't talk on bus
    	printf("block_read failed\n");
    }

    hdev->i32rawTemperature = (int32_t)buf[0]<<16 | (int32_t)buf[1]<<8 | (int32_t)buf[2];
    hdev->i32rawTemperature= (hdev->i32rawTemperature&0x800000) ? (0xFF000000|hdev->i32rawTemperature) : hdev->i32rawTemperature;
}

float AP_Baro_SPL0601::_spl0601_get_pressure(_spl0601_t *hdev)
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

float AP_Baro_SPL0601::_spl0601_get_temperature(_spl0601_t *hdev)
{
    float fTCompensate;
    float fTsc;

    fTsc = hdev->i32rawTemperature / (float)hdev->i32kT;
    fTCompensate =  hdev->calib_param.c0 * 0.5 + hdev->calib_param.c1 * fTsc;
    return fTCompensate;
}
