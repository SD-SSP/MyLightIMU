# ICM20600 & AK09918 Lightweight ESP I2C Driver

A high-performance, bare-metal style C++ driver written from the ground up to solve memory allocation bottlenecks and scale errors found in legacy libraries. Built specifically for high-speed sensor applications running on the ESP32 using the groove ICM20600 + AK09918 9DOF IMU.

## Why this was coded (The Performance Bottleneck)

The popular **Seeed Studio Grove - IMU 9DOF (ICM20600+AK09918)** module [Seeed Studio Wiki](https://wiki.seeedstudio.com/Grove-IMU_9DOF-lcm20600+AK09918/) leaves developers with a hardware constraint: its internal SPI interface pads are hard-wired/soldered to the power lines by default [Seeed Studio Wiki](https://wiki.seeedstudio.com/Grove-IMU_9DOF_v2.0/). This configuration forces the use of the **I2C interface** for all data communications [Seeed Studio Wiki](https://wiki.seeedstudio.com/Grove-IMU_9DOF-lcm20600+AK09918/).

For applications demanding high sample rates (such as drone flight controllers, robotics, or real-time vibration tracking), legacy Arduino libraries face severe limitations:
* **The `Wire.h`:** Standard Arduino drivers utilizing the legacy `Wire.h` library on the ESP32 carry immense overhead due to blocking architectures and redundant memory allocation hooks. They struggle to push past a **~1000 Hz** polling frequency.
* **The Solution:** By stripping out `Wire.h` and building directly upon the native **ESP-IDF v5 Master Bus HAL (`i2c_master.h`)**, this driver successfully achieved a rock-solid, stable **2500 Hz sample rate** during testing.

### Can it go even higher?
**Yes.** The absolute theoretical limit of the I2C Fast-Mode Plus protocol running at **400 kHz** (the maximum rated speed for the ICM20600) allows for higher data throughput if the application loop bypasses secondary math routines. This lightweight driver was designed specifically to give your ESP32 the headroom needed to push the silicon to its physical limits.

## Sidenotes

- The AK09918 cant do more than 100Hz by desing. - sadge
- For simplicity the library handels the master bus, if you need to add other sensors to the I2C bus just 'init()' this and 'getBusHandle()' to pass to the others. - if i though of this before this could be done better
- I could do this with ICM and AK being separate objects but oh well. - maybe will do this later
- This could be better.

## Key Architectural Advantages
- **Zero-Allocation Data Streams:** Eliminates loose, wasteful pointer arguments and runtime allocations in your high-frequency loop, reducing stack overhead to zero.
- **Framework Agnostic (Pure C++):** Contains zero dependencies on `Arduino.h` or FreeRTOS headers. Fully portable between pure ESP-IDF projects and Arduino IDE environments.
- **On-Demand Inline Math:** True g-force, DPS (Degrees Per Second), and Tesla values are calculated on the fly only when explicitly requested.
- **ESP-IDF v5 Native Core:** Built using the non-blocking Master Bus peripheral architecture (`i2c_master_transmit_receive`). 
- **Advanced Bus Sharing:** Exposes a native bus handle getter, allowing users to safely share the same I2C bus across multiple external devices.

## Hardware Connections (ESP32 Example)

| Sensor Pin | ESP32 Pin | Description |
| :--- | :--- | :---: |
| **VCC** | `3.3V` | Power Supply (3.3V) |
| **GND** | `GND` | Ground Reference |
| **SDA** | `GPIO 21` | Default I2C Data Line |
| **SCL** | `GPIO 22` | Default I2C Clock Line |

## Quick Start Example (Arduino IDE / ESP-IDF)

```cpp
#include <Arduino.h>
#include "ICM20600_AK09918_I2C.h"

ICM_AK_I2C device;
ICM_Data   imuData;

void setup() {
  Serial.begin(115200);

  // Initialize utilizing the modern configuration struct pattern
  init_cfg cfg{
    .clock_speed_hz = 400000,
    .icm_mode       = ICM_MODE::FULL_ON, // High-performance mode recommended for loops > 100Hz
    .accel_scale    = ICM_ACCEL_SCALE::G4
  };

  if (!device.init(cfg)) {
    Serial.println("Hardware Initialization Failed! Check Wiring/IDs.");
    while (1) { delay(10); } 
  }
  Serial.println("IMU Online & Validated.");
}

void loop() {
  // Query raw data packets directly from the hardware registers
  if (device.readICM(&imuData)) {
    
    // Evaluate conversions instantly inside your code block using inline helpers
    float ax = device.getAX(imuData);
    float ay = device.getAY(imuData);
    float az = device.getAZ(imuData);
    float temp = device.getTemp(imuData); // Retrieved with 0 microseconds of extra I2C overhead

    Serial.printf("AX: %.4f g | AY: %.4f g | AZ: %.4f g | Temp: %.2f C\n", ax, ay, az, temp);
  }
  delay(10); // Running at a stable frequency
}
```

## Crucial Power Mode Optimization Warnings
This library exposes **`ICM_MODE::ACCEL_LOW_POWER`**, **`ICM_MODE::STANDBY`**, and **`ICM_MODE::SLEEP`** configurations.

* **Low Power Mode:** Operates the internal analog converters using hardware duty-cycling. If you attempt to poll the I2C registers faster than **100 Hz** in this mode, you will pull corrupted or flatlined voltage residuals (**0.05g**) from the internal sleep cache. For high-speed applications (**>= 100 Hz**), always execute under **`ICM_MODE::FULL_ON`**.
* **Standby Mode:** Leaves the internal gyroscope drive motor spinning to handle instant wake-ups (**<= 5ms**) while keeping the measurement lines locked out to conserve current.

## License

This project is licensed under the GNU General Public License v3.0 - see the [LICENSE](LICENSE) file for details.
