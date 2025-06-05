#include <LSM6DSV16BXSensor.h>
LSM6DSV16BXSensor AccGyr(&Wire, LSM6DSV16BX_I2C_ADD_L);

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  // Initialize LSM6DSV16BX.
  AccGyr.begin();

  // Enable accelerometer
  AccGyr.Enable_X();

  // Enable QVAR
  if (AccGyr.QVAR_Enable() != 0) {
    Serial.println("Error during initialization of QVAR");
    while (1);
  }
  Serial.println("LSM6DSV16BX QVAR Demo");
}

void loop()
{
  uint8_t qvar_status;
  float qvar_data;
  // Check if QVAR data is ready
  AccGyr.QVAR_GetStatus(&qvar_status);
  if (qvar_status) {
    // Get QVAR data
    AccGyr.QVAR_GetData(&qvar_data);
    Serial.println(qvar_data);
  }
  delay(100);
}
