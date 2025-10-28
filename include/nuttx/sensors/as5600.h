
#ifndef __SENSORS_AS5600_H
#define __SENSORS_AS5600_H

#include <nuttx/config.h>

#if defined(CONFIG_I2C)  && defined(CONFIG_SENSORS_AS5600)


int as5600_register(FAR const char *devpath, FAR struct i2c_master_s *i2c);
#endif /* CONFIG_I2C && CONFIG_SENSORS_AS5600 */

#endif /* __SENSORS_AS5600_H */