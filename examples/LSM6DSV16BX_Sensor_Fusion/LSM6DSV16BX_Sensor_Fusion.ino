/*
 * You can display the quaternion values with a 3D model connecting for example to this link:
 * https://adafruit.github.io/Adafruit_WebSerial_3DModelViewer/
 */

// Tested with X-NUCLEO-IKS01A3
#include <LSM6DSV16BXSensor.h>

#define ALGO_FREQ  120U /* Algorithm frequency 120Hz */
#define ALGO_PERIOD  (1000U / ALGO_FREQ) /* Algorithm period [ms] */
unsigned long startTime, elapsedTime;

#define INT_1 A5

//I2C_ADD_L for compatibility with X-NUCLEO-IKS01A3
LSM6DSV16BXSensor AccGyr(&Wire, LSM6DSV16BX_I2C_ADD_L);

uint8_t status = 0;
uint32_t k = 0;

uint8_t tag = 0;
float quaternions[4] = {0};

void setup()
{
  Serial.begin(115200);

  while (!Serial) {
    yield();
  }

  //PULL DOWN for compatibility with X-NUCLEO-IKS01A3
  pinMode(INT_1, OUTPUT);
  digitalWrite(INT_1, LOW);

  Wire.begin();


  // Initialize LSM6DSV16BX.
  AccGyr.begin();
  AccGyr.Enable_X();
  AccGyr.Enable_G();

  // Enable Sensor Fusion

  status |= AccGyr.Enable_Sensor_Fusion();
  status |= AccGyr.Set_X_FS(4);
  status |= AccGyr.Set_G_FS(2000);
  status |= AccGyr.Set_X_ODR(120.0f);
  status |= AccGyr.Set_G_ODR(120.0f);
  status |= AccGyr.Set_SFLP_ODR(120.0f);
  status |= AccGyr.Enable_Rotation_Vector();
  status |= AccGyr.FIFO_Set_Mode(LSM6DSV16BX_STREAM_MODE);

  if (status != LSM6DSV16BX_OK) {
    Serial.println("LSM6DSV16BX Sensor failed to init/configure");
    while (1);
  }
  Serial.println("LSM6DSV16BX SFLP Demo");
}

void loop()
{
  uint16_t fifo_samples;
  // Get start time of loop cycle
  startTime = millis();

  // Check the number of samples inside FIFO
  if (AccGyr.FIFO_Get_Num_Samples(&fifo_samples) != LSM6DSV16BX_OK) {
    Serial.println("LSM6DSV16BX Sensor failed to get number of samples inside FIFO");
    while (1);
  }

  // Read the FIFO if there is one stored sample
  if (fifo_samples > 0) {
    for (int i = 0; i < fifo_samples; i++) {
      AccGyr.FIFO_Get_Tag(&tag);
      if (tag == 0x13u) {
        AccGyr.FIFO_Get_Rotation_Vector(&quaternions[0]);

        // Print Quaternion data
        Serial.print("Quaternion: ");
        Serial.print(-(quaternions[3]), 4);
        Serial.print(", ");
        Serial.print(quaternions[0], 4);
        Serial.print(", ");
        Serial.print(quaternions[1], 4);
        Serial.print(", ");
        Serial.println(-(quaternions[2]), 4);

        // Compute the elapsed time within loop cycle and wait
        elapsedTime = millis() - startTime;

        if ((long)(ALGO_PERIOD - elapsedTime) > 0) {
          delay(ALGO_PERIOD - elapsedTime);
        }
      }
    }
  }
  delay(100);
}
