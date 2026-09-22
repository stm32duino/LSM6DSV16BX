
#include <LSM6DSV16BXSensor.h>

#define INT_1 A5

//I2C_ADD_L for compatibility with X-NUCLEO-IKS01A3
LSM6DSV16BXSensor LSM6DSV16BX(&Wire, LSM6DSV16BX_I2C_ADD_L);

//Interrupts.
volatile int mems_event = 0;

char report[256];

void INT1Event_cb();
void sendOrientation();

void setup()
{
  // Initlialize serial.
  Serial.begin(115200);
  delay(1000);

  // Initlialize Led.
  pinMode(LED_BUILTIN, OUTPUT);

  // Initlialize i2c.
  Wire.begin();

  // Enable INT1 pin.
  attachInterrupt(INT_1, INT1Event_cb, RISING);

  // Initlialize components.
  LSM6DSV16BX.begin();
  LSM6DSV16BX.Enable_X();

  // Enable 6D Orientation.
  LSM6DSV16BX.Enable_6D_Orientation(LSM6DSV16BX_INT1_PIN);
}

void loop()
{
  if (mems_event) {
    mems_event = 0;
    LSM6DSV16BX_Event_Status_t status;
    LSM6DSV16BX.Get_X_Event_Status(&status);

    if (status.D6DOrientationStatus) {
      // Send 6D Orientation
      sendOrientation();

      // Led blinking.
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}

void INT1Event_cb()
{
  mems_event = 1;
}

void sendOrientation()
{
  uint8_t xl = 0;
  uint8_t xh = 0;
  uint8_t yl = 0;
  uint8_t yh = 0;
  uint8_t zl = 0;
  uint8_t zh = 0;

  LSM6DSV16BX.Get_6D_Orientation_XL(&xl);
  LSM6DSV16BX.Get_6D_Orientation_XH(&xh);
  LSM6DSV16BX.Get_6D_Orientation_YL(&yl);
  LSM6DSV16BX.Get_6D_Orientation_YH(&yh);
  LSM6DSV16BX.Get_6D_Orientation_ZL(&zl);
  LSM6DSV16BX.Get_6D_Orientation_ZH(&zh);

  if (xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 1 && zh == 0) {
    sprintf(report, "\r\n  ________________  " \
            "\r\n |                | " \
            "\r\n |  *             | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |________________| \r\n");
  }

  else if (xl == 1 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 0) {
    sprintf(report, "\r\n  ________________  " \
            "\r\n |                | " \
            "\r\n |             *  | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |________________| \r\n");
  }

  else if (xl == 0 && yl == 0 && zl == 0 && xh == 1 && yh == 0 && zh == 0) {
    sprintf(report, "\r\n  ________________  " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |  *             | " \
            "\r\n |________________| \r\n");
  }

  else if (xl == 0 && yl == 1 && zl == 0 && xh == 0 && yh == 0 && zh == 0) {
    sprintf(report, "\r\n  ________________  " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |             *  | " \
            "\r\n |________________| \r\n");
  }

  else if (xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 1) {
    sprintf(report, "\r\n  __*_____________  " \
            "\r\n |________________| \r\n");
  }

  else if (xl == 0 && yl == 0 && zl == 1 && xh == 0 && yh == 0 && zh == 0) {
    sprintf(report, "\r\n  ________________  " \
            "\r\n |________________| " \
            "\r\n    *               \r\n");
  }

  else {
    sprintf(report, "None of the 6D orientation axes is set in accelerometer.\r\n");
  }

  Serial.print(report);
}
