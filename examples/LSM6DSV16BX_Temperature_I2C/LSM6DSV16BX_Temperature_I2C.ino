//NOTE: this example isn't compatible with Arduino Uno
#include <LSM6DSV16BXSensor.h>

#define INT_1 A5

LSM6DSV16BXSensor accGyr(&Wire, LSM6DSV16BX_I2C_ADD_L); //I2C_ADD_L per compatibilità con X-NUCLEO-IKS01A3

float odr;

void setup()
{
  // Led.
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(INT_1, OUTPUT);
  digitalWrite(INT_1, LOW);

  // Initialize serial for output.
  Serial.begin(115200);

  // Initialize I2C bus.
  Wire.begin();

  accGyr.begin();

  accGyr.Enable_X();
  accGyr.Enable_G();


}

void loop()
{
  // Led blinking.
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);

  float temp;
  accGyr.Get_Temp(&temp);
  Serial.print("temp[C]: ");
  Serial.println(temp);
  Serial.println("- - - - - - - - - - - - - - - -");

  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}
