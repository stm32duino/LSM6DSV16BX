// Tested with X-NUCLEO-IKS01A3
#include <LSM6DSV16BXSensor.h>

#define INT_1 A5

//I2C_ADD_L for compatibility with X-NUCLEO-IKS01A3
LSM6DSV16BXSensor LSM6DSV16BX(&Wire, LSM6DSV16BX_I2C_ADD_L);

//Interrupts.
volatile int mems_event = 0;

void INT1Event_cb();

void setup()
{
  Serial.begin(115200);
  delay(1000);
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();

  //Interrupts.
  attachInterrupt(INT_1, INT1Event_cb, RISING);

  // Initlialize components.
  LSM6DSV16BX.begin();
  LSM6DSV16BX.Enable_X();

  // Enable Single Tap Detection.
  LSM6DSV16BX.Enable_Single_Tap_Detection(LSM6DSV16BX_INT1_PIN);
}

void loop()
{
  if (mems_event) {
    mems_event = 0;
    LSM6DSV16BX_Event_Status_t status;
    LSM6DSV16BX.Get_X_Event_Status(&status);
    if (status.TapStatus) {

      // Led blinking.
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      Serial.println("Single Tap Detected!");
    }
  }
}

void INT1Event_cb()
{
  mems_event = 1;
}
