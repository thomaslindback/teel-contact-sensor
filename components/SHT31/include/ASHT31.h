/*!
 *  @file Adafruit_SHT31.h
 *
 *  This is a library for the SHT31 Digital Humidity & Temp Sensor
 *
 *  Designed specifically to work with the  Digital Humidity & Temp Sensor
 *  -----> https://www.adafruit.com/product/2857
 *
 *  These sensors use I2C to communicate, 2 pins are required to interface
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *
 *  BSD license, all text above must be included in any redistribution
 */

#pragma once

#include <inttypes.h>
#include "driver/i2c.h"

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define SHT31_I2C_MASTER_SCL              CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define SHT31_I2C_MASTER_SDA              CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define SHT31_I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define SHT31_I2C_MASTER_FREQ_HZ          100000                     /*!< I2C master clock frequency */
#define SHT31_I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define SHT31_I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define SHT31_I2C_MASTER_TIMEOUT_MS       1000

#define SHT31_DEFAULT_ADDR 0x44 /**< SHT31 Default Address */
#define SHT31_MEAS_HIGHREP_STRETCH 0x2C06 /**< Measurement High Repeatability with Clock Stretch Enabled */
#define SHT31_MEAS_MEDREP_STRETCH 0x2C0D /**< Measurement Medium Repeatability with Clock Stretch Enabled */
#define SHT31_MEAS_LOWREP_STRETCH 0x2C10 /**< Measurement Low Repeatability with Clock Stretch Enabled*/
#define SHT31_MEAS_HIGHREP 0x2400 /**< Measurement High Repeatability with Clock Stretch Disabled */
#define SHT31_MEAS_MEDREP  0x240B /**< Measurement Medium Repeatability with Clock Stretch Disabled */
#define SHT31_MEAS_LOWREP  0x2416 /**< Measurement Low Repeatability with Clock Stretch Disabled */
#define SHT31_READSTATUS 0xF32D   /**< Read Out of Status Register */
#define SHT31_CLEARSTATUS 0x3041  /**< Clear Status */
#define SHT31_SOFTRESET 0x30A2    /**< Soft Reset */
#define SHT31_HEATEREN 0x306D     /**< Heater Enable */
#define SHT31_HEATERDIS 0x3066    /**< Heater Disable */
#define SHT31_REG_HEATER_BIT 0x0d /**< Status Register Heater Bit */

namespace sht31 {

class ASHT31 {
public:
  ASHT31();
  ~ASHT31();

  bool begin(uint8_t i2caddr = SHT31_DEFAULT_ADDR, i2c_port_t master_port_num = I2C_NUM_0);
  float readTemperature(void);
  float readHumidity(void);
  bool readBoth(float *temperature_out, float *humidity_out);
  uint16_t readStatus(void);
  void reset(void);
  void heater(bool h);
  bool isHeaterEnabled();

private:
  float humidity;
  float temp;
  bool readTempHum(void);
  esp_err_t writeCommand(uint16_t cmd);

  i2c_port_t _master_port_num;
  uint8_t _i2caddr;
  esp_err_t register_read(uint16_t reg_addr, uint8_t *data, size_t len);
};

}