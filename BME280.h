#pragma once
#include <mbed.h>

#include "bme280_defs.h"

class BME280 {
 public:
  BME280(I2C &i2c, int address);

  int init();

  enum class Config { WEATHER_MONITORING, INDOOR_NAVIGATION };
  int setConfig(Config);

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
  int setNormalMode(StandbyTime);
  int setForcedMode();
  int setSleepMode();

  Kernel::Clock::duration_u32 getUpdateDelay() const;
  int updateData();

  uint32_t getPressure() const { return data_.pressure; }
  int32_t getTemperature() const { return data_.temperature; }
  uint32_t getHumidity() const { return data_.humidity; }

 private:
  static BME280_INTF_RET_TYPE read(uint8_t reg_addr, uint8_t *reg_data,
                                   uint32_t len, void *intf_ptr);
  static BME280_INTF_RET_TYPE write(uint8_t reg_addr, const uint8_t *reg_data,
                                    uint32_t len, void *intf_ptr);
  static void delayUs(uint32_t period, void *intf_ptr);

  I2C &i2c_;
  int addr_;
  struct bme280_dev dev_;
  struct bme280_data data_;
};
