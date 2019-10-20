/*
  minimal test program for ICM20789 baro with IMU on SPI and baro on I2C
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <utility>
#include <stdio.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_HAL::OwnPtr<AP_HAL::Device> i2c_dev;

#define HAL_COMPASS_HMC5843_I2C_ADDR 0x1E
#define HMC5843_REG_ID_A 0x0A

void setup(void);
void loop(void);

bool read_registers(uint8_t first_reg, uint8_t *recv, uint32_t recv_len)
{
    //first_reg |= 0x80;
    return i2c_dev->transfer(&first_reg, 1, recv, recv_len);
}

bool block_read(uint8_t reg, uint8_t *buf, uint32_t size)
{
    return read_registers(reg, buf, size);
}

static void i2c_init(void)
{
    if (!i2c_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        AP_HAL::panic("Failed to get semaphore");
    }

    uint8_t id[3];
    memset(id, 0, sizeof(id));
    if (!block_read(HMC5843_REG_ID_A, id, 3)) {
        // can't talk on bus
    	printf("block_read failed\n");
    }
    if (id[0] != 'H' ||
        id[1] != '4' ||
        id[2] != '3') {
        // not a HMC5x83 device
    	printf("id error\n");
    }

    printf("id: %c%c%c\n", id[0], id[1], id[2]);

    hal.scheduler->delay(1);

    i2c_dev->get_semaphore()->give();
}

void setup()
{
    printf("HMC5843 test\n");

    AP_BoardConfig{}.init();

    hal.scheduler->delay(1000);

    i2c_dev = std::move(hal.i2c_mgr->get_device(1, HAL_COMPASS_HMC5843_I2C_ADDR));

    while (true) {
        i2c_init();
        hal.scheduler->delay(1000);
    }
}


void loop()
{
}

AP_HAL_MAIN();
