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
#include "ICM20600_AK09918_I2C.h"

#include <cstdint>
#include <driver/i2c_master.h>

#include "esp_err.h"
#include "esp_rom_sys.h"

ICM_AK_I2C::~ICM_AK_I2C() {
  if (AK_HANDLE != nullptr) {
    i2c_master_bus_rm_device(AK_HANDLE);
    AK_HANDLE = nullptr;
  }
  if (ICM_HANDLE != nullptr) {
    i2c_master_bus_rm_device(ICM_HANDLE);
    ICM_HANDLE = nullptr;
  }
  if (bus_owner && BUS_HANDLE != nullptr) {
    i2c_del_master_bus(BUS_HANDLE);
    BUS_HANDLE = nullptr;
  }
}

bool ICM_AK_I2C::init(const init_cfg &cfg) {

  // Master bus config
  i2c_master_bus_config_t bus_cfg = {};
  bus_cfg.i2c_port = cfg.i2c_port;
  bus_cfg.sda_io_num = cfg.sda_pin;
  bus_cfg.scl_io_num = cfg.scl_pin;
  bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_cfg.glitch_ignore_cnt = 7;
  bus_cfg.flags.enable_internal_pullup = true;

  // ICM device config
  i2c_device_config_t icm_cfg = {};
  icm_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  icm_cfg.device_address = cfg.icm_address;
  icm_cfg.scl_speed_hz = cfg.clock_speed_hz;

  // AK device config
  i2c_device_config_t ak_cfg = {};
  ak_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  ak_cfg.device_address = cfg.ak_address;
  ak_cfg.scl_speed_hz = cfg.clock_speed_hz;

  // I2C bus allocation
  if(i2c_new_master_bus(&bus_cfg, &BUS_HANDLE) != ESP_OK) return false;;
  this->bus_owner = true;
  // register ICM device
  if(i2c_master_bus_add_device(BUS_HANDLE, &icm_cfg, &ICM_HANDLE) != ESP_OK) goto fail_bus;
  // register AK device
  if(i2c_master_bus_add_device(BUS_HANDLE, &ak_cfg, &AK_HANDLE) != ESP_OK) goto fail_icm;

  std::uint8_t tx_reg;
  std::uint8_t rx_val;

  // verify ICM20600 identity
  tx_reg = cfg.ICM_WHO_AM_I;
  if (i2c_master_transmit_receive(ICM_HANDLE, &tx_reg, 1, &rx_val, 1, -1) != ESP_OK || 
      rx_val != cfg.ICM_DEVICE_ID) {
    goto fail_all; // abort if chip is missing or wrong ID
  }

  // verify AK09918 identity 
  tx_reg = cfg.AK_WIA_2;
  if (i2c_master_transmit_receive(AK_HANDLE, &tx_reg, 1, &rx_val, 1, -1) != ESP_OK || 
      rx_val != cfg.AK_DEVICE_ID) {
    goto fail_all; // abort if chip is missing or wrong ID
  }

  if(!setICMMode(cfg.icm_mode) || !setAccelScale(cfg.accel_scale) || !setGyroScale(cfg.gyro_scale) || !setAKmode(cfg.ak_mode)) goto fail_all;

  switch (cfg.accel_scale) {
    case ICM_ACCEL_SCALE::G2:  this->a_scale = 1.0f / 16384.0f; break;
    case ICM_ACCEL_SCALE::G4:  this->a_scale = 1.0f / 8192.0f;  break;
    case ICM_ACCEL_SCALE::G8:  this->a_scale = 1.0f / 4096.0f;  break;
    case ICM_ACCEL_SCALE::G16: this->a_scale = 1.0f / 2048.0f;  break;
    default:                   this->a_scale = 1.0f / 8192.0f;  break;
  }

  switch (cfg.gyro_scale) {
    case ICM_GYRO_SCALE::DPS_250:  this->g_scale = 1.0f / 131.0f;  break;
    case ICM_GYRO_SCALE::DPS_500:  this->g_scale = 1.0f / 65.5f;   break;
    case ICM_GYRO_SCALE::DPS_1000: this->g_scale = 1.0f / 32.8f;   break;
    case ICM_GYRO_SCALE::DPS_2000: this->g_scale = 1.0f / 16.4f;   break;
    default:                       this->g_scale = 1.0f / 65.5f;   break;
  }

  return true;

fail_all:
  i2c_master_bus_rm_device(AK_HANDLE);
  AK_HANDLE = nullptr;

fail_icm:
  i2c_master_bus_rm_device(ICM_HANDLE);
  ICM_HANDLE = nullptr;

fail_bus:
  i2c_del_master_bus(BUS_HANDLE);
  BUS_HANDLE = nullptr;
  this->bus_owner = false;

  return false;
}

bool ICM_AK_I2C::init_shared(const init_cfg &cfg, i2c_master_bus_handle_t shared_bus) {
  if (shared_bus == nullptr) return false;

  this->bus_owner = false;
  
  this->BUS_HANDLE = shared_bus;

  i2c_device_config_t icm_cfg = {};
  icm_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  icm_cfg.device_address  = cfg.icm_address;
  icm_cfg.scl_speed_hz    = cfg.clock_speed_hz;

  i2c_device_config_t ak_cfg = {};
  ak_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  ak_cfg.device_address  = cfg.ak_address;
  ak_cfg.scl_speed_hz    = cfg.clock_speed_hz;

  if(i2c_master_bus_add_device(BUS_HANDLE, &icm_cfg, &ICM_HANDLE) != ESP_OK) return false;
  if(i2c_master_bus_add_device(BUS_HANDLE, &ak_cfg, &AK_HANDLE) != ESP_OK) goto fail_icm;

  std::uint8_t tx_reg;
  std::uint8_t rx_val;

  // verify ICM20600 identity
  tx_reg = cfg.ICM_WHO_AM_I;
  if (i2c_master_transmit_receive(ICM_HANDLE, &tx_reg, 1, &rx_val, 1, -1) != ESP_OK || 
      rx_val != cfg.ICM_DEVICE_ID) {
    goto fail_all; // abort if chip is missing or wrong ID
  }

  // verify AK09918 identity 
  tx_reg = cfg.AK_WIA_2;
  if (i2c_master_transmit_receive(AK_HANDLE, &tx_reg, 1, &rx_val, 1, -1) != ESP_OK || 
      rx_val != cfg.AK_DEVICE_ID) {
    goto fail_all; // abort if chip is missing or wrong ID
  }

  if(!setICMMode(cfg.icm_mode) || !setAccelScale(cfg.accel_scale) || !setGyroScale(cfg.gyro_scale) || !setAKmode(cfg.ak_mode)) goto fail_all;

  switch (cfg.accel_scale) {
    case ICM_ACCEL_SCALE::G2:  this->a_scale = 1.0f / 16384.0f; break;
    case ICM_ACCEL_SCALE::G4:  this->a_scale = 1.0f / 8192.0f;  break;
    case ICM_ACCEL_SCALE::G8:  this->a_scale = 1.0f / 4096.0f;  break;
    case ICM_ACCEL_SCALE::G16: this->a_scale = 1.0f / 2048.0f;  break;
    default:                   this->a_scale = 1.0f / 8192.0f;  break;
  }

  switch (cfg.gyro_scale) {
    case ICM_GYRO_SCALE::DPS_250:  this->g_scale = 1.0f / 131.0f;  break;
    case ICM_GYRO_SCALE::DPS_500:  this->g_scale = 1.0f / 65.5f;   break;
    case ICM_GYRO_SCALE::DPS_1000: this->g_scale = 1.0f / 32.8f;   break;
    case ICM_GYRO_SCALE::DPS_2000: this->g_scale = 1.0f / 16.4f;   break;
    default:                       this->g_scale = 1.0f / 65.5f;   break;
  }

  return true;

  fail_all:
    i2c_master_bus_rm_device(AK_HANDLE);
    AK_HANDLE = nullptr;

  fail_icm:
    i2c_master_bus_rm_device(ICM_HANDLE);
    ICM_HANDLE = nullptr;

  return false;
}

bool ICM_AK_I2C::setICMMode(ICM_MODE mode) {
  std::uint8_t pwr1_val = 0x00;
  std::uint8_t pwr2_val = 0x00;

  switch (mode) {
    case ICM_MODE::SLEEP:
      pwr1_val = 0x40; // Set SLEEP bit (Bit 6)
      pwr2_val = 0x00;
      break;
            
    case ICM_MODE::STANDBY:
      pwr1_val = 0x10; // Set GYRO_STANDBY bit (Bit 4)
      pwr2_val = 0x38; // Disable Accel X, Y, Z channels (Bits 5-3)
      break;
            
    case ICM_MODE::ACCEL_LOW_POWER:
      pwr1_val = 0x20; // Set CYCLE bit (Bit 5)
      pwr2_val = 0x07; // Disable Gyro X, Y, Z channels (Bits 2-0)
      break;
            
    case ICM_MODE::FULL_ON:
      pwr1_val = 0x00; // Clear all sleep / cycle bits
      pwr2_val = 0x00; // Enable all sensor axes channels
      break;
    }

    std::uint8_t cmd1[2] = {ICM20600_REG::PWR_MGMT_1, pwr1_val};
    std::uint8_t cmd2[2] = {ICM20600_REG::PWR_MGMT_2, pwr2_val};

    if (i2c_master_transmit(ICM_HANDLE, cmd1, sizeof(cmd1), -1) != ESP_OK) return false;
    if (i2c_master_transmit(ICM_HANDLE, cmd2, sizeof(cmd2), -1) != ESP_OK) return false;

    return true;
}

bool ICM_AK_I2C::setAKmode(AK_MODE mode) {
  std::uint8_t cmd[2] = {AK09918_REG::CNTL2 , static_cast<uint8_t>(mode)};

  if(i2c_master_transmit(AK_HANDLE, cmd, sizeof(cmd), -1) != ESP_OK) return false;

  return true;
}

bool ICM_AK_I2C::setAccelScale(ICM_ACCEL_SCALE scale) {
  std::uint8_t cmd[2] = {ICM20600_REG::ACCEL_FS_SEL , static_cast<uint8_t>(scale)};

  if(i2c_master_transmit(ICM_HANDLE, cmd, sizeof(cmd), -1) != ESP_OK) return false;

  return true;
}

bool ICM_AK_I2C::setGyroScale(ICM_GYRO_SCALE scale) {
  std::uint8_t cmd[2] = {ICM20600_REG::GYRO_CONFIG , static_cast<uint8_t>(scale)};

  if(i2c_master_transmit(ICM_HANDLE, cmd, sizeof(cmd), -1) != ESP_OK)  return false;

  return true;
}

bool ICM_AK_I2C::readICM(ICM_Data *data) {
  if (data == nullptr) [[unlikely]] return false;

  std::uint8_t raw_buffer[14];
  std::uint8_t XOUT_H = ICM20600_REG::ACCEL_XOUT_H;
  if(i2c_master_transmit_receive(ICM_HANDLE, &XOUT_H, 1, raw_buffer, sizeof(raw_buffer), -1) != ESP_OK) [[unlikely]] {
    return false;
  };

  data->raw_ax = static_cast<int16_t>((raw_buffer[0] << 8) | raw_buffer[1]);
  data->raw_ay = static_cast<int16_t>((raw_buffer[2] << 8) | raw_buffer[3]);
  data->raw_az = static_cast<int16_t>((raw_buffer[4] << 8) | raw_buffer[5]);
  data->raw_gx = static_cast<int16_t>((raw_buffer[8] << 8) | raw_buffer[9]);
  data->raw_gy = static_cast<int16_t>((raw_buffer[10]<< 8) | raw_buffer[11]);
  data->raw_gz = static_cast<int16_t>((raw_buffer[12]<< 8) | raw_buffer[13]);

  return true;
}

bool ICM_AK_I2C::readAccel(Accel_Data *data) {
  if (data == nullptr) [[unlikely]] return false;

  std::uint8_t raw_buffer[6];
  std::uint8_t reg_start = ICM20600_REG::ACCEL_XOUT_H;

  if (i2c_master_transmit_receive(ICM_HANDLE, &reg_start, 1, raw_buffer, sizeof(raw_buffer), -1) != ESP_OK) [[unlikely]] {
    return false;
  }

  data->raw_x = static_cast<int16_t>((raw_buffer[0] << 8) | raw_buffer[1]);
  data->raw_y = static_cast<int16_t>((raw_buffer[2] << 8) | raw_buffer[3]);
  data->raw_z = static_cast<int16_t>((raw_buffer[4] << 8) | raw_buffer[5]);

  return true;
}

bool ICM_AK_I2C::readGyro(Gyro_Data *data) {
  if (data == nullptr) [[unlikely]] return false;

  std::uint8_t raw_buffer[6];
  std::uint8_t reg_start = ICM20600_REG::GYRO_XOUT_H;

  if (i2c_master_transmit_receive(ICM_HANDLE, &reg_start, 1, raw_buffer, sizeof(raw_buffer), -1) != ESP_OK) [[unlikely]] {
    return false;
  }

  data->raw_x = static_cast<int16_t>((raw_buffer[0] << 8) | raw_buffer[1]);
  data->raw_y = static_cast<int16_t>((raw_buffer[2] << 8) | raw_buffer[3]);
  data->raw_z = static_cast<int16_t>((raw_buffer[4] << 8) | raw_buffer[5]);

  return true;
}

bool ICM_AK_I2C::readAK(AK_Data *data) {
  if (data == nullptr) [[unlikely]] return false;

  std::uint8_t st1_reg = AK09918_REG::ST1;
  std::uint8_t status1 = 0;

  if(i2c_master_transmit_receive(AK_HANDLE, &st1_reg, 1, &status1, 1, -1) != ESP_OK) [[unlikely]] return false;
  if (!(status1 & 0x01)) [[unlikely]] return false;

  std::uint8_t ak_reg_start = AK09918_REG::HXL;
  std::uint8_t raw_buffer[8];
  if (i2c_master_transmit_receive(AK_HANDLE, &ak_reg_start, 1, raw_buffer, sizeof(raw_buffer), -1) != ESP_OK) [[unlikely]] {
    return false;
  }

  data->raw_x = static_cast<int32_t>((raw_buffer[1] << 8) | raw_buffer[0]);
  data->raw_y = static_cast<int32_t>((raw_buffer[3] << 8) | raw_buffer[2]);
  data->raw_z = static_cast<int32_t>((raw_buffer[5] << 8) | raw_buffer[4]);

  return true;
}

bool ICM_AK_I2C::readTemp(Temp_Data *data) {
  if (data == nullptr) [[unlikely]] return false;

  std::uint8_t raw_buffer[2];
  std::uint8_t reg_start = ICM20600_REG::TEMP_OUT_H;

  if(i2c_master_transmit_receive(ICM_HANDLE, &reg_start, 1, raw_buffer, sizeof(raw_buffer), -1) != ESP_OK) [[unlikely]] return false;

  data->raw_temp = static_cast<int16_t>((raw_buffer[0] << 8) | raw_buffer[1]);

  return true;
}

bool ICM_AK_I2C::reset() {
  if(ICM_HANDLE == nullptr) [[unlikely]] return false;

  std::uint8_t cmd[2] = {ICM20600_REG::PWR_MGMT_1, 0x80};
  if(i2c_master_transmit(ICM_HANDLE, cmd, sizeof(cmd), -1) != ESP_OK) return false;

  esp_rom_delay_us(10000);
  return true;
}