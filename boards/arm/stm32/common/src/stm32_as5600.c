

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/sensors/as5600.h>
#include <nuttx/i2c/i2c_master.h>
#include <stdio.h>
#include <debug.h>
#include "stm32_i2c.h"

int stm32_as5600_initialize(int bus)
{
    struct i2c_master_s *i2c;
    int ret;

    sninfo("Initializing BMP180!\n");

    /* Initialize I2C */

    i2c = stm32_i2cbus_initialize(bus);

    if (!i2c)
    {
        return -ENODEV;
    }


    ret = as5600_register("/dev/as5600", i2c);
    if (ret < 0)
    {
        snerr("ERROR: Error registering /dev/as5600, errno:%d \n", ret);
    }

    return ret;
}