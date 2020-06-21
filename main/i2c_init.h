#ifndef _I2C_INIT_H
#define _I2C_INIT_H

#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
//#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

int i2c_port_num;

esp_err_t i2c_common_init(int I2C_MASTER_NUM, int I2C_MASTER_SCL_IO, int I2C_MASTER_SDA_IO, uint32_t I2C_MASTER_FREQ_HZ);

#endif
