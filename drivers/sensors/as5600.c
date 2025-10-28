/****************************************************************************
 * drivers/sensors/as5600.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* Character driver for the Freescale AS56001 Barometer Sensor */

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>
#include <unistd.h>
#include <inttypes.h>
#include <stdlib.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/as5600.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_AS5600)

struct as5600_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* AS5600 I2C address */
  int freq;                     /* AS5600 Frequency */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static ssize_t as5600_read(FAR struct file *filep, FAR uint8_t *buffer, size_t buflen);
static ssize_t as5600_write(FAR struct file *filep, FAR const uint8_t *buffer, size_t buflen);
static int as5600_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_as5600fops =
    {
        NULL,         /* open */
        NULL,         /* close */
        NULL,         /* read */
        NULL,         /* write */
        NULL,         /* seek */
        as5600_ioctl, /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: as5600_read
 ****************************************************************************/

static ssize_t as5600_read_reg(FAR struct as5600_dev_s *priv, uint8_t regaddr, uint8_t *regval)
{
  int ret = OK;
  struct i2c_config_s config;

  /* Set up the I2C configuration */

  config.frequency = priv->frequency;
  config.address = priv->addr;
  config.addrlen = 7;

  /* Write the register address */

  ret = i2c_write(priv->i2c, &config, &regaddr, sizeof(regaddr));
  if (ret < 0)
  {
    snerr("ERROR: i2c_write failed: %d\n", ret);
    return ret;
  }

  /* Restart and read 8 bits from the register */

  ret = i2c_read(priv->i2c, &config, regval, sizeof(uint8_t));
  if (ret < 0)
  {
    snerr("ERROR: i2c_read failed: %d\n", ret);
    return ret;
  }

  sninfo("addr: %02x value: %02x ret: %d\n", regaddr, *regval, ret);
  return ret;
}
}

/****************************************************************************
 * Name: as5600_write
 ****************************************************************************/

static ssize_t as5600_write_reg(FAR struct as5600_dev_s *priv, uint8_t regaddr, uint8_t regval)
{

  struct i2c_config_s config;
  int ret = OK;

  sninfo("addr: %02x value: %02x\n", regaddr, regval);

  /* Set up the I2C configuration */

  config.frequency = priv->frequency;
  config.address = priv->addr;
  config.addrlen = 7;

  /* Set up a 2-byte message to send */

  buffer[0] = regaddr;
  buffer[1] = regval;

  /* Write the register address and value */

  ret = i2c_write(priv->i2c, &config, buffer, 2);
  if (ret < 0)
  {
    snerr("ERROR: i2c_write failed: %d\n", ret);
  }

  return ret;
}

static int as5600_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct as5600_dev_s *priv = inode->i_private;

  as5600_reg_io_s *reg_io = (FAR struct as5600_reg_io_s *)((uintptr_t)arg);
  int ret = 0;

  switch (cmd)
  {
  case AS5600_IOC_READ_REG:
  {
    
    if (reg_io->data == NULL)
    {
        malloc(reg_io, sizeof(uint8_t) * buflen);
    }
    uint8_t data;
    for (size_t i = 0; i < reg_io->buflen; i++)
    {

      ret = as5600_read_reg(priv, &reg_io->regval, data);
      if (ret < 0)
      {
        snerr("AS5600_IOC_READ_REG failed: %d\n", ret);
        return ret;
      }
      reg_io->data[i] = data;
    }
    break;
  }
  
  case AS5600_IOC_WRITE_REG:
  {
    ret = as5600_write_reg(priv, reg_io->regval, reg - io->buflen);
    if (ret < 0)
    {
      snerr("AS5600_IOC_WRITE_REG failed: %d\n", ret);
      return ret;
    }
    break;
  }

  default:
  {
    snerr("Unrecognized cmd: %d\n", cmd);
    ret = -ENOTTY;
    break;
  }
  return ret;
}
/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: as5600_register
 *
 * Description:
 *   Register the AS5600 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/press0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             AS5600
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int as5600_register(FAR const char *devpath, FAR struct i2c_master_s *i2c)
{
  FAR struct as5600_dev_s *priv;
  int ret;

  /* Initialize the AS5600 device structure */

  priv = (FAR struct as5600_dev_s *)kmm_malloc(sizeof(struct as5600_dev_s));
  if (!priv)
  {
    snerr("ERROR: Failed to allocate instance\n");
    return -ENOMEM;
  }

  priv->i2c = i2c;
  priv->addr = AS5600_ADDR;
  priv->freq = AS5600_FREQ;

  /* Check Device ID */

  /* Register the character driver */

  ret = register_driver(devpath, &g_as5600fops, 0666, priv);
  if (ret < 0)
  {
    snerr("ERROR: Failed to register driver: %d\n", ret);
    kmm_free(priv);
  }

  sninfo("AS5600 driver loaded successfully!\n");
  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_AS5600 */