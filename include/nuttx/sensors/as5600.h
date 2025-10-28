
#ifndef __SENSORS_AS5600_H
#define __SENSORS_AS5600_H

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_I2C)  && defined(CONFIG_SENSORS_AS5600)

#define AS5600_ADDR 0x36 /* 7-bit address for AS5600 */

#ifdef CONFIG_SENSORS_AS5600_FREQUENCY
#define AS5600_FREQ CONFIG_SENSORS_AS5600_FREQUENCY
#else
#define AS5600_FREQ 400000 /* 400kHz */
#endif

#define AS5600_REG_ZMCO               0x00        /**< written times register */
#define AS5600_REG_ZPOS_H             0x01        /**< start position register high */
#define AS5600_REG_ZPOS_L             0x02        /**< start position register low */
#define AS5600_REG_MPOS_H             0x03        /**< stop position register high */
#define AS5600_REG_MPOS_L             0x04        /**< stop position register low */
#define AS5600_REG_MANG_H             0x05        /**< maximum angle register high */
#define AS5600_REG_MANG_L             0x06        /**< maximum angle register low */
#define AS5600_REG_CONF_H             0x07        /**< conf register high */
#define AS5600_REG_CONF_L             0x08        /**< conf register low */
#define AS5600_REG_RAW_ANGLE_H        0x0C        /**< raw angle register high */
#define AS5600_REG_RAW_ANGLE_L        0x0D        /**< raw angle register low */
#define AS5600_REG_ANGLE_H            0x0E        /**< angle register high */
#define AS5600_REG_ANGLE_L            0x0F        /**< angle register low */
#define AS5600_REG_STATUS             0x0B        /**< status register */
#define AS5600_REG_AGC                0x1A        /**< automatic gain control register */
#define AS5600_REG_MAGNITUDE_H        0x1B        /**< magnitude register high */
#define AS5600_REG_MAGNITUDE_L        0x1C        /**< magnitude register low */
#define AS5600_REG_BURN               0xFF        /**< burn register */

int as5600_register(FAR const char *devpath, FAR struct i2c_master_s *i2c);

#define AS5600_IOC_BASE          _SNIOC(0x00B0)
#define _AS5600_IOC(X)           (AS5600_IOC_BASE + X)

//sequential numbering from AS5600_IOC_BASE
#define AS5600_IOC_SET_START_POS    _AS5600_IOC(0x0001) 
#define AS5600_IOC_SET_STOP_POS     _AS5600_IOC(0x0002) 
#define AS5600_IOC_SET_MAX_ANGLE    _AS5600_IOC(0x0003) 
#define AS5600_IOC_GET_RAW_ANGLE    _AS5600_IOC(0x0004) 
#define AS5600_IOC_GET_ANGLE        _AS5600_IOC(0x0005) 
#define AS5600_IOC_GET_STATUS       _AS5600_IOC(0x0006) 
#define AS5600_IOC_GET_AGC         _AS5600_IOC(0x0007) 
#define AS5600_IOC_GET_MAGNITUDE     _AS5600_IOC(0x0008) 

#define AS5600_IOC_READ_REG       _AS5600_IOC(0x0009)
#define AS5600_IOC_WRITE_REG      _AS5600_IOC(0x000A)



struct as5600_reg_io_s
{
  uint8_t reg;
  uint8_t * data;
  uint8_t buflen;
};




#endif /* CONFIG_I2C && CONFIG_SENSORS_AS5600 */

#endif /* __SENSORS_AS5600_H */