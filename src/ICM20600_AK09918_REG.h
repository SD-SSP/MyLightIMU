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
#ifndef ICM20600_AK09918_REG_h
#define ICM20600_AK09918_REG_h

#include <cstdint>

// ICM20600 register addreses
namespace ICM20600_REG {
  constexpr std::uint8_t ADDR         = 0x69;
  constexpr std::uint8_t ACCEL_XOUT_H = 0x3B;
  constexpr std::uint8_t ACCEL_FS_SEL = 0x1C;
  constexpr std::uint8_t PWR_MGMT_1   = 0x6B;
  constexpr std::uint8_t PWR_MGMT_2   = 0x6C;
  constexpr std::uint8_t GYRO_CONFIG  = 0x1B;
  constexpr std::uint8_t GYRO_XOUT_H  = 0x43;
  constexpr std::uint8_t WHO_AM_I     = 0x75;
  constexpr std::uint8_t DEVICE_ID    = 0x11;
  constexpr std::uint8_t TEMP_OUT_H   = 0x41;
}

// AK09918 register addreses
namespace AK09918_REG {
  constexpr std::uint8_t ADDR      = 0x0C;
  constexpr std::uint8_t ST1       = 0x10;
  constexpr std::uint8_t HXL       = 0x11;
  constexpr std::uint8_t CNTL2     = 0x31;
  constexpr std::uint8_t WIA1      = 0x00;
  constexpr std::uint8_t WIA2      = 0x01;
  constexpr std::uint8_t DEVICE_ID = 0x0C;
}
#endif