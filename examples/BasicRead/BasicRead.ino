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
    while (1);
  }
  Serial.println("IMU Online & Validated.");
}

void loop() {
  // Query only the data packets your application script explicitly requires
  if (device.readICM(&imuData)) {
    
    // Evaluate conversions instantly inside your code block using inline helpers
    float ax = device.getAX(imuData);
    float ay = device.getAY(imuData);
    float az = device.getAZ(imuData);

    Serial.printf("AX: %.4f g | AY: %.4f g | AZ: %.4f g\n", ax, ay, az);
  }
  delay(10); // Running at a stable, non-blocking frequency
}