#pragma once

#include "AP_Baro_Backend.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>
#include <Filter/Filter.h>

#ifndef HAL_BARO_SPL06_I2C_ADDR
#define HAL_BARO_SPL06_I2C_ADDR 0x76
#endif

#ifndef HAL_BARO_SPL06_I2C_ADDR2
#define HAL_BARO_SPL06_I2C_ADDR2 0x77
#endif

class AP_Baro_SPL0601 : public AP_Baro_Backend
{
public:
	AP_Baro_SPL0601(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);

    /* AP_Baro public interface: */
    void update() override;

    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);
    
private:
    struct _spl0601_calib_param_t {
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

    typedef struct _spl0601_t {
        struct _spl0601_calib_param_t calib_param;/**<calibration data*/
        int32_t i32rawPressure;
        int32_t i32rawTemperature;
        int32_t i32kP;
        int32_t i32kT;
    } _spl0601_t;

    bool _init();

    void _timer();

    bool     _block_read(uint8_t reg, uint8_t *buf, uint32_t size);
    void     _spl0601_write(_spl0601_t *hdev, uint8_t hwadr, uint8_t regadr, uint8_t val);
    uint8_t  _spl0601_read(_spl0601_t *hdev, uint8_t hwadr, uint8_t regadr);
    void     _spl0601_start_continuous(_spl0601_t *hdev, uint8_t mode);
    void     _spl0601_get_calib_param(_spl0601_t *hdev);
    void     _spl0601_rateset(_spl0601_t *hdev, uint8_t iSensor, uint8_t u8SmplRate, uint8_t u8OverSmpl);
    void     _spl0601_get_raw_pressure(_spl0601_t *hdev);
    void     _spl0601_get_raw_temp(_spl0601_t *hdev);
    float    _spl0601_get_pressure(_spl0601_t *hdev);
    float    _spl0601_get_temperature(_spl0601_t *hdev);

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
    LowPassFilterFloat             _lowpass_filter;
    bool                           _has_sample;
    uint8_t                        _instance;
    _spl0601_t                     _spl0601;
    float                          _pressure;
    float                          _temperature;
};
