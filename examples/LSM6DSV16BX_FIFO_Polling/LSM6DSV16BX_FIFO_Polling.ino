// Tested with X-NUCLEO-IKS01A3
#include <LSM6DSV16BXSensor.h>

#define SENSOR_ODR 104.0f // In Hertz
#define ACC_FS 2 // In g
#define GYR_FS 2000 // In dps
#define MEASUREMENT_TIME_INTERVAL (1000.0f/SENSOR_ODR) // In ms
#define FIFO_SAMPLE_THRESHOLD 199
#define FLASH_BUFF_LEN 8192

#define INT_1 A5 // MCU input pin connected to sensor INT1 output pin 

//I2C_ADD_L for compatibility with X-NUCLEO-IKS01A3
LSM6DSV16BXSensor LSM6DSV16BX(&Wire, LSM6DSV16BX_I2C_ADD_L);

volatile uint8_t fullFlag = 0; // FIFO full flag
uint8_t status = 0;
unsigned long timestamp_count = 0;
bool acc_available = false;
bool gyr_available = false;
int32_t acc_value[3];
int32_t gyr_value[3];
char buff[FLASH_BUFF_LEN];
uint32_t pos = 0;

void Read_FIFO_Data();

void setup()
{

  Serial.begin(115200);
  Wire.begin();

  // Initialize LSM6DSV16BX.
  LSM6DSV16BX.begin();
  status |= LSM6DSV16BX.Enable_X();
  status |= LSM6DSV16BX.Enable_G();

  // Configure ODR and FS of the acc and gyro
  status |= LSM6DSV16BX.Set_X_ODR(SENSOR_ODR);
  status |= LSM6DSV16BX.Set_X_FS(ACC_FS);
  status |= LSM6DSV16BX.Set_G_ODR(SENSOR_ODR);
  status |= LSM6DSV16BX.Set_G_FS(GYR_FS);

  // Configure FIFO BDR for acc and gyro
  status |= LSM6DSV16BX.FIFO_Set_X_BDR(SENSOR_ODR);
  status |= LSM6DSV16BX.FIFO_Set_G_BDR(SENSOR_ODR);

  // Set FIFO in Continuous mode
  status |= LSM6DSV16BX.FIFO_Set_Mode(LSM6DSV16BX_STREAM_MODE);

  if (status != LSM6DSV16BX_OK) {
    Serial.println("LSM6DSV16BX Sensor failed to init/configure");
    while (1);
  }
  Serial.println("LSM6DSV16BX FIFO Demo");
}

void loop()
{
  uint16_t fifo_samples;

  // Check the number of samples inside FIFO
  if (LSM6DSV16BX.FIFO_Get_Num_Samples(&fifo_samples) != LSM6DSV16BX_OK) {
    Serial.println("LSM6DSV16BX Sensor failed to get number of samples inside FIFO");
    while (1);
  }

  // If we reach the threshold we can empty the FIFO
  if (fifo_samples > FIFO_SAMPLE_THRESHOLD) {

    // Empty the FIFO
    Read_FIFO_Data();

    // Print FIFO data
    Serial.print(buff);
  }
}

void Read_FIFO_Data()
{
  uint16_t i;
  uint16_t samples_to_read;

  // Check the number of samples inside FIFO
  if (LSM6DSV16BX.FIFO_Get_Num_Samples(&samples_to_read) != LSM6DSV16BX_OK) {
    Serial.println("LSM6DSV16BX Sensor failed to get number of samples inside FIFO");
    while (1);
  }

  for (i = 0; i < samples_to_read; i++) {
    uint8_t tag;

    // Check the FIFO tag
    if (LSM6DSV16BX.FIFO_Get_Tag(&tag) != LSM6DSV16BX_OK) {
      Serial.println("LSM6DSV16BX Sensor failed to get tag");
      while (1);
    }
    switch (tag) {
      // If we have a gyro tag, read the gyro data
      case 1: {
          if (LSM6DSV16BX.FIFO_Get_G_Axes(gyr_value) != LSM6DSV16BX_OK) {
            Serial.println("LSM6DSV16BX Sensor failed to get gyroscope data");
            while (1);
          }
          gyr_available = true;
          break;
        }

      // If we have an acc tag, read the acc data
      case 2: {
          if (LSM6DSV16BX.FIFO_Get_X_Axes(acc_value) != LSM6DSV16BX_OK) {
            Serial.println("LSM6DSV16BX Sensor failed to get accelerometer data");
            while (1);
          }
          acc_available = true;
          break;
        }

      // We can discard other tags
      default: {
          break;
        }
    }
    // If we have the measurements of both acc and gyro, we can store them with timestamp
    if (acc_available && gyr_available) {
      int num_bytes;
      num_bytes = snprintf(&buff[pos], (FLASH_BUFF_LEN - pos), "%lu AccZ: %d AccY: %d AccX: %d GyrX: %d GyrY: %d GyrZ: %d\r\n", (unsigned long)((float)timestamp_count * MEASUREMENT_TIME_INTERVAL), (int)acc_value[0], (int)acc_value[1], (int)acc_value[2], (int)gyr_value[0], (int)gyr_value[1], (int)gyr_value[2]);
      pos += num_bytes;
      timestamp_count++;
      acc_available = false;
      gyr_available = false;
    }
  }
  // We can add the termination character to the string, so we are ready to print it on hyper-terminal
  buff[pos] = '\0';
  pos = 0;
}
