#include "driver/i2c_types.h"
/*
 * Copyright (C) 2026 [SD-SSP] <szymondziewierski9@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://gnu.org>.
 */

// library for groove ICM20600 + AK09918 9-DOF IMU using ESP32 I2C drivers
#ifndef ICM20600_AK09918_I2C_h
#define ICM20600_AK09918_I2C_h

#include <cstdint>
#include <driver/i2c_master.h>

#include "soc/gpio_num.h"
#include "hal/i2c_types.h"

#include "ICM20600_AK09918_REG.h"

// ICM20600 modes
enum class ICM_MODE : std::uint8_t {
  SLEEP           = 1, // pwr1_val = 0x40, pwr2_val = 0x00
  STANDBY         = 2, // pwr1_val = 0x10, pwr2_val = 0x38
  ACCEL_LOW_POWER = 3, // pwr1_val = 0x20, pwr2_val = 0x07 dont use if sampling rate >= 100Hz
  FULL_ON         = 4, // pwr1_val = 0x00, pwr2_val = 0x00
};

// ICM20600 accel scales
enum class ICM_ACCEL_SCALE : std::uint8_t {
  G2  = 0x00, // +-2g
  G4  = 0x08, // +-4g
  G8  = 0x10, // +-8g
  G16 = 0x18 // +-16g
};

// ICM20600 gyro scales
enum class ICM_GYRO_SCALE : std::uint8_t {
  DPS_250  = 0x00, // +-250dps
  DPS_500  = 0x08, // +-500dps
  DPS_1000 = 0x10, // +-1000dps
  DPS_2000 = 0x18  // +-2000dps
};

// AK09918 modes
enum class AK_MODE : std::uint8_t {
  PD = 0x00, // power down
  SM = 0x01, // single measurement
  C1 = 0x02, // continous 10Hz
  C2 = 0x04, // continous 20Hz
  C3 = 0x06, // continous 50Hz
  C4 = 0x08, // continous 100Hz
  ST = 0x10  // self test
};

// default device config that needs to be passed to init()
struct init_cfg {
  i2c_port_t i2c_port = I2C_NUM_0;
  gpio_num_t sda_pin = GPIO_NUM_21;
  gpio_num_t scl_pin = GPIO_NUM_22;
  std::uint32_t clock_speed_hz = 400000;

  std::uint8_t icm_address = ICM20600_REG::ADDR;
  std::uint8_t ak_address = AK09918_REG::ADDR;

  ICM_MODE icm_mode = ICM_MODE::FULL_ON;
  ICM_ACCEL_SCALE accel_scale = ICM_ACCEL_SCALE::G4;
  ICM_GYRO_SCALE gyro_scale = ICM_GYRO_SCALE::DPS_500;
  AK_MODE ak_mode = AK_MODE::C4;

  // dont mess with these ones below
  std::uint8_t ICM_WHO_AM_I = ICM20600_REG::WHO_AM_I;
  std::uint8_t ICM_DEVICE_ID = ICM20600_REG::DEVICE_ID;
  std::uint8_t AK_WIA_1 = AK09918_REG::WIA1;
  std::uint8_t AK_WIA_2 = AK09918_REG::WIA2;
  std::uint8_t AK_DEVICE_ID = AK09918_REG::DEVICE_ID;
};

struct ICM_Data {
  std::int16_t raw_ax, raw_ay, raw_az;
  std::int16_t raw_gx, raw_gy, raw_gz;
};

struct Accel_Data {
  std::int16_t raw_x, raw_y, raw_z;
};

struct Gyro_Data {
  std::int16_t raw_x, raw_y, raw_z;
};

struct AK_Data {
  std::int16_t raw_x, raw_y, raw_z;
};

struct Temp_Data {
  std::int16_t raw_temp;
};

class ICM_AK_I2C {
private:
  bool bus_owner;

  i2c_master_bus_handle_t BUS_HANDLE = nullptr;
  i2c_master_dev_handle_t ICM_HANDLE = nullptr;
  i2c_master_dev_handle_t  AK_HANDLE = nullptr;

  float a_scale = 0.0f;
  float g_scale = 0.0f;

  static constexpr float m_scale = 0.15f;
  static constexpr float t_scale = 1.0 / 326.8f;

public:
  ICM_AK_I2C() = default;
  ~ICM_AK_I2C();

  bool init(const init_cfg &cfg = {});
  bool init_shared(const init_cfg &cfg, i2c_master_bus_handle_t shared_bus);

  bool setICMMode(ICM_MODE mode);
  bool setAKmode(AK_MODE mode);
  bool setAccelScale(ICM_ACCEL_SCALE scale);
  bool setGyroScale(ICM_GYRO_SCALE scale);

  bool readICM(ICM_Data *data);
  bool readAccel(Accel_Data *data);
  bool readGyro(Gyro_Data *data);
  bool readAK(AK_Data *data);

  bool readTemp(Temp_Data *data);

  bool reset();

  i2c_master_bus_handle_t getBusHandle() const { return BUS_HANDLE; }

  inline float getAX(const ICM_Data &d)  const { return static_cast<float>(d.raw_ax) * a_scale; }
  inline float getAY(const ICM_Data &d)  const { return static_cast<float>(d.raw_ay) * a_scale; }
  inline float getAZ(const ICM_Data &d)  const { return static_cast<float>(d.raw_az) * a_scale; }

  inline float getGX(const ICM_Data &d)  const { return static_cast<float>(d.raw_gx) * a_scale; }
  inline float getGY(const ICM_Data &d)  const { return static_cast<float>(d.raw_gy) * a_scale; }
  inline float getGZ(const ICM_Data &d)  const { return static_cast<float>(d.raw_gz) * a_scale; }

  inline float getAX(const Accel_Data &d) const { return static_cast<float>(d.raw_x) * a_scale; }
  inline float getAY(const Accel_Data &d) const { return static_cast<float>(d.raw_y) * a_scale; }
  inline float getAZ(const Accel_Data &d) const { return static_cast<float>(d.raw_z) * a_scale; }

  inline float getGX(const Gyro_Data &d)  const { return static_cast<float>(d.raw_x) * g_scale; }
  inline float getGY(const Gyro_Data &d)  const { return static_cast<float>(d.raw_y) * g_scale; }
  inline float getGZ(const Gyro_Data &d)  const { return static_cast<float>(d.raw_z) * g_scale; }

  inline float getMX(const AK_Data &d)    const { return static_cast<float>(d.raw_x) * m_scale; }
  inline float getMY(const AK_Data &d)    const { return static_cast<float>(d.raw_y) * m_scale; }
  inline float getMZ(const AK_Data &d)    const { return static_cast<float>(d.raw_z) * m_scale; }

  inline float getT(const Temp_Data &d)  const { return static_cast<float>(d.raw_temp) * t_scale + 25.0f; }
};
#endif // ICM20600_AK09918_I2C_h



