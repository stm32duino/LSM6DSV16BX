// Tested with X-NUCLEO-IKS01A3
#include <LSM6DSV16BXSensor.h>

//I2C_ADD_L for compatibility with X-NUCLEO-IKS01A3
LSM6DSV16BXSensor LSM6DSV16BX(&Wire, LSM6DSV16BX_I2C_ADD_L);


void setup()
{
  // Initlialize serial.
  Serial.begin(115200);
  delay(1000);

  // Initlialize Led.
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize i2c.
  Wire.begin();

  // Initlialize components.
  if (LSM6DSV16BX.begin() != LSM6DSV16BX_OK) {
    Serial.println("Failed to initialize the sensor!");
    while (1);
  }

  // Run the self test for accelerometer and gyroscope:
  // LSM6DSV16BX_XL_ST_DISABLE,
  // LSM6DSV16BX_XL_ST_POSITIVE,
  // LSM6DSV16BX_XL_ST_NEGATIVE
  uint8_t xTestType = LSM6DSV16BX_XL_ST_POSITIVE;
  uint8_t gTestType = LSM6DSV16BX_XL_ST_POSITIVE;
  LSM6DSV16BXStatusTypeDef result = LSM6DSV16BX.Test_IMU(xTestType, gTestType);

  // Check the result of the self test
  if (result == LSM6DSV16BX_OK) {
    Serial.println("Self test passed!");
  } else {
    Serial.println("Self test failed!");
  }
}

void loop()
{
  // Nothing to do in the loop
}
