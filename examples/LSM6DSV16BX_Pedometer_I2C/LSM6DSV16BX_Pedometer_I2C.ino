// Tested with X-NUCLEO-IKS01A3
#include <LSM6DSV16BXSensor.h>

#define INT_1 A5

//I2C_ADD_L for compatibility with X-NUCLEO-IKS01A3
LSM6DSV16BXSensor LSM6DSV16BX(&Wire, LSM6DSV16BX_I2C_ADD_L);

//Interrupts.
volatile int mems_event = 0;
uint16_t step_count = 0;
uint32_t previous_tick;
char report[256];

void INT1Event_cb();


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

  // Enable Pedometer.
  LSM6DSV16BX.Enable_Pedometer(LSM6DSV16BX_INT1_PIN);

  previous_tick = millis();
}

void loop()
{
  if (mems_event) {
    mems_event = 0;
    LSM6DSV16BX_Event_Status_t status;
    LSM6DSV16BX.Get_X_Event_Status(&status);

    if (status.StepStatus) {
      // Led blinking.
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);

      LSM6DSV16BX.Get_Step_Count(&step_count);
      snprintf(report, sizeof(report), "Step counter: %d", step_count);
      Serial.println(report);
    }
  }
  // Print the step counter in any case every 3000 ms
  uint32_t current_tick = millis();
  if ((current_tick - previous_tick) >= 3000) {
    LSM6DSV16BX.Get_Step_Count(&step_count);
    snprintf(report, sizeof(report), "Step counter: %d", step_count);
    Serial.println(report);
    previous_tick = millis();
  }

}

void INT1Event_cb()
{
  mems_event = 1;
}
