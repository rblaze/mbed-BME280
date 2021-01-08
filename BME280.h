#pragma once
#include <mbed.h>

#include "bme280_defs.h"

class BME280 {
public:
  BME280(I2C &i2c, int address);

  int init();

  enum class Config { WEATHER_MONITORING, INDOOR_NAVIGATION };
  int set_config(Config);

  enum class StandbyTime {
    MS_0_5,
    MS_10,
    MS_20,
    MS_62_5,
    MS_125,
    MS_250,
    MS_500,
    MS_1000
  };
  int set_normal_mode(StandbyTime);
  int set_forced_mode();
  int set_sleep_mode();

  int update_data();
  uint32_t pressure() { return data_.pressure; }
  int32_t temperature() { return data_.temperature; }
  uint32_t humidity() { return data_.humidity; }

private:
  static BME280_INTF_RET_TYPE read(
      uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
  static BME280_INTF_RET_TYPE write(
      uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
  static void delay_us(uint32_t period, void *intf_ptr);

  I2C &i2c_;
  int addr_;
  struct bme280_dev dev_;
  struct bme280_data data_;
};
