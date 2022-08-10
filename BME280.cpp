#include "BME280.h"

#include "bme280_driver.h"

BME280::BME280(I2C &i2c, int address) : i2c_{i2c}, addr_{address} {
  dev_.intf_ptr = this;
  dev_.intf = BME280_I2C_INTF;
  dev_.read = &BME280::read;
  dev_.write = &BME280::write;
  dev_.delay_us = &BME280::delayUs;
}

int BME280::init() { return bme280_init(&dev_); }

int BME280::setConfig(BME280::Config config) {
  switch (config) {
    case Config::WEATHER_MONITORING:
      dev_.settings.osr_h = BME280_OVERSAMPLING_1X;
      dev_.settings.osr_p = BME280_OVERSAMPLING_1X;
      dev_.settings.osr_t = BME280_OVERSAMPLING_1X;
      dev_.settings.filter = BME280_FILTER_COEFF_OFF;
      break;
    case Config::INDOOR_NAVIGATION:
      dev_.settings.osr_h = BME280_OVERSAMPLING_1X;
      dev_.settings.osr_p = BME280_OVERSAMPLING_16X;
      dev_.settings.osr_t = BME280_OVERSAMPLING_2X;
      dev_.settings.filter = BME280_FILTER_COEFF_16;
      break;
  }

  auto settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL |
                      BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

  return bme280_set_sensor_settings(settings_sel, &dev_);
}

int BME280::setSleepMode() {
  return bme280_set_sensor_mode(BME280_SLEEP_MODE, &dev_);
}

int BME280::setForcedMode() {
  return bme280_set_sensor_mode(BME280_FORCED_MODE, &dev_);
}

int BME280::setNormalMode(BME280::StandbyTime standby) {
  switch (standby) {
    case StandbyTime::MS_0_5:
      dev_.settings.standby_time = BME280_STANDBY_TIME_0_5_MS;
      break;
    case StandbyTime::MS_10:
      dev_.settings.standby_time = BME280_STANDBY_TIME_10_MS;
      break;
    case StandbyTime::MS_20:
      dev_.settings.standby_time = BME280_STANDBY_TIME_20_MS;
      break;
    case StandbyTime::MS_62_5:
      dev_.settings.standby_time = BME280_STANDBY_TIME_62_5_MS;
      break;
    case StandbyTime::MS_125:
      dev_.settings.standby_time = BME280_STANDBY_TIME_125_MS;
      break;
    case StandbyTime::MS_250:
      dev_.settings.standby_time = BME280_STANDBY_TIME_250_MS;
      break;
    case StandbyTime::MS_500:
      dev_.settings.standby_time = BME280_STANDBY_TIME_500_MS;
      break;
    case StandbyTime::MS_1000:
      dev_.settings.standby_time = BME280_STANDBY_TIME_1000_MS;
      break;
  }

  auto ret = bme280_set_sensor_settings(BME280_STANDBY_SEL, &dev_);
  if (ret == 0) {
    ret = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev_);
  }

  return ret;
}

Kernel::Clock::duration_u32 BME280::getUpdateDelay() const {
  uint32_t delay_ms = bme280_cal_meas_delay(&dev_.settings);

  return Kernel::Clock::duration_u32(delay_ms);
}

int BME280::updateData() {
  return bme280_get_sensor_data(BME280_ALL, &data_, &dev_);
}

BME280_INTF_RET_TYPE BME280::read(uint8_t reg_addr, uint8_t *reg_data,
                                  uint32_t len, void *intf_ptr) {
  auto owner = static_cast<BME280 *>(intf_ptr);
  auto reg = reinterpret_cast<const char *>(&reg_addr);
  auto data = reinterpret_cast<char *>(reg_data);

  auto res = owner->i2c_.write(owner->addr_, reg, sizeof(reg_addr));
  if (res == 0) {
    res = owner->i2c_.read(owner->addr_, data, len);
  }

  return res;
}

BME280_INTF_RET_TYPE BME280::write(uint8_t reg_addr, const uint8_t *reg_data,
                                   uint32_t len, void *intf_ptr) {
  auto owner = static_cast<BME280 *>(intf_ptr);
  auto reg = reinterpret_cast<const char *>(&reg_addr);
  auto data = reinterpret_cast<const char *>(reg_data);

  auto res = owner->i2c_.write(owner->addr_, reg, sizeof(reg_addr), true);
  if (res != 0) {
    owner->i2c_.stop();
  } else {
    res = owner->i2c_.write(owner->addr_, data, len);
  }

  return res;
}

void BME280::delayUs(uint32_t period, void *) {
  // Driver only sleeps for 1ms or 2ms, use ThisThread::sleep_for()
  auto delay = std::chrono::microseconds(period);
  auto u32delay =
      std::chrono::duration_cast<Kernel::Clock::duration_u32>(delay);

  if (u32delay < delay) {
    u32delay = Kernel::Clock::duration_u32{1};
  }

  ThisThread::sleep_for(u32delay);
}
