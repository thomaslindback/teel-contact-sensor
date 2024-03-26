/*!
 *  @file Adafruit_SHT31.cpp
 *
 *  @mainpage Adafruit SHT31 Digital Humidity & Temp Sensor
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for the SHT31 Digital Humidity & Temp Sensor
 *
 *  Designed specifically to work with the SHT31 Digital sensor from Adafruit
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/2857
 *
 *  These sensors use I2C to communicate, 2 pins are required to interface
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  @section author Author
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *
 *  @section license License
 *
 *  BSD license, all text above must be included in any redistribution
 */

#include <cmath>
#include "ASHT31.h"
#include "esp_log.h"

static const char* TAG = "ASHT31-sensor";

using namespace sht31;

sht31::ASHT31::ASHT31() {}

sht31::ASHT31::~ASHT31() {
  esp_err_t i2c_driver_delete(i2c_port_t i2c_num);
}

bool sht31::ASHT31::begin(uint8_t i2caddr, i2c_port_t master_port_num) {
  _master_port_num = master_port_num;
  _i2caddr = i2caddr;

  i2c_config_t conf = {.mode = I2C_MODE_MASTER,
                       .sda_io_num = SHT31_I2C_MASTER_SDA,
                       .scl_io_num = SHT31_I2C_MASTER_SCL,
                       .sda_pullup_en = GPIO_PULLUP_ENABLE,
                       .scl_pullup_en = GPIO_PULLUP_ENABLE,
                       .master{.clk_speed = SHT31_I2C_MASTER_FREQ_HZ},
                       .clk_flags = 0};

  esp_err_t err = i2c_param_config(_master_port_num, &conf);
  if (err != ESP_OK) {
    ESP_LOGI(TAG, "Error installing i2c config %s", esp_err_to_name(err));
  }
  err = i2c_driver_install(_master_port_num, conf.mode,
                           SHT31_I2C_MASTER_RX_BUF_DISABLE,
                           SHT31_I2C_MASTER_TX_BUF_DISABLE, 0);
  if (err != ESP_OK) {
    ESP_LOGI(TAG, "Error installing i2c driver");
  }
  humidity = std::nanf("-99");
  temp = std::nanf("-99");

  reset();
  return readStatus() != 0xFFFF;
}

esp_err_t ASHT31::register_read(uint16_t reg_addr, uint8_t* data, size_t len) {
  uint8_t addr[2];
  addr[0] = reg_addr >> 8;
  addr[1] = reg_addr & 0xFF;

  return i2c_master_write_read_device(
      _master_port_num, _i2caddr, addr, 2, data, len,
      SHT31_I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

uint16_t ASHT31::readStatus(void) {
  uint8_t data[2];
  esp_err_t err = register_read(SHT31_READSTATUS, data, 2);
  if (err != ESP_OK) {
    ESP_LOGI(TAG, "Error reading status %s", esp_err_to_name(err));
    return -99;
  }

  uint16_t stat = data[0];
  stat <<= 8;
  stat |= data[1];
  return stat;
}

void ASHT31::reset(void) {
  if (!writeCommand(SHT31_SOFTRESET)) {
    ESP_LOGI(TAG, "Error reset");
  }
  vTaskDelay(pdMS_TO_TICKS(10));
}

void ASHT31::heater(bool h) {
  if (h)
    writeCommand(SHT31_HEATEREN);
  else
    writeCommand(SHT31_HEATERDIS);
  vTaskDelay(pdMS_TO_TICKS(1));
}

bool ASHT31::isHeaterEnabled() {
  uint16_t regValue = readStatus();
  return (bool)bitRead(regValue, SHT31_REG_HEATER_BIT);
}

float sht31::ASHT31::readTemperature(void) {
  if (!readTempHum())
    return std::nanf("-99");

  return temp;
}

float sht31::ASHT31::readHumidity(void) {
  if (!readTempHum())
    return std::nanf("-99");

  return humidity;
}

bool sht31::ASHT31::readBoth(float* temperature_out, float* humidity_out) {
  if (!readTempHum()) {
    *temperature_out = *humidity_out = std::nanf("-99");
    return false;
  }

  *temperature_out = temp;
  *humidity_out = humidity;
  return true;
}

static uint8_t crc8(const uint8_t* data, int len) {
  /*
   *
   * CRC-8 formula from page 14 of SHT spec pdf
   *
   * Test data 0xBE, 0xEF should yield 0x92
   *
   * Initialization data 0xFF
   * Polynomial 0x31 (x8 + x5 +x4 +1)
   * Final XOR 0x00
   */

  const uint8_t POLYNOMIAL(0x31);
  uint8_t crc(0xFF);

  for (int j = len; j; --j) {
    crc ^= *data++;

    for (int i = 8; i; --i) {
      crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
    }
  }
  return crc;
}

bool sht31::ASHT31::readTempHum(void) {
  uint8_t readbuffer[6];

  esp_err_t err = register_read(SHT31_MEAS_HIGHREP, readbuffer, 6);

  if (err != ESP_OK)
    return false;

  vTaskDelay(pdMS_TO_TICKS(20));

  if (readbuffer[2] != crc8(readbuffer, 2) ||
      readbuffer[5] != crc8(readbuffer + 3, 2))
    return false;

  int32_t stemp = (int32_t)(((uint32_t)readbuffer[0] << 8) | readbuffer[1]);
  // simplified (65536 instead of 65535) integer version of:
  // temp = (stemp * 175.0f) / 65535.0f - 45.0f;
  stemp = ((4375 * stemp) >> 14) - 4500;
  temp = (float)stemp / 100.0f;

  uint32_t shum = ((uint32_t)readbuffer[3] << 8) | readbuffer[4];
  // simplified (65536 instead of 65535) integer version of:
  // humidity = (shum * 100.0f) / 65535.0f;
  shum = (625 * shum) >> 12;
  humidity = (float)shum / 100.0f;

  return true;
}

esp_err_t sht31::ASHT31::writeCommand(uint16_t command) {
  ESP_LOGI(TAG, "Status %i", readStatus());
  uint8_t cmd[2];

  cmd[0] = command >> 8;
  cmd[1] = command & 0xFF;

  return i2c_master_write_to_device(
      _master_port_num, _i2caddr, cmd, 2,
      SHT31_I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}
