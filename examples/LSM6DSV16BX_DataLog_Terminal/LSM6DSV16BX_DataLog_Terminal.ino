
#include <LSM6DSV16BXSensor.h>

#define INT_1 A5

//I2C_ADD_L for compatibility with X-NUCLEO-IKS01A3
LSM6DSV16BXSensor sensor(&Wire, LSM6DSV16BX_I2C_ADD_L);

int32_t accGyro[3];

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);

  //PULL DOWN for compatibility with X-NUCLEO-IKS01A3
  pinMode(INT_1, OUTPUT);
  digitalWrite(INT_1, LOW);

  Serial.begin(115200);
  Wire.begin();

  sensor.begin();
  sensor.Enable_X();
  sensor.Enable_G();
}

void loop()
{
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
  delay(250);

  sensor.Get_X_Axes(accGyro);

  Serial.print("Accel   X[mg]:");
  Serial.print(accGyro[0]);
  Serial.print("  Y[mg]:");
  Serial.print(accGyro[1]);
  Serial.print("  Z[mg]:");
  Serial.println(accGyro[2]);

  sensor.Get_G_Axes(accGyro);
  Serial.print("AngRate X[mdps]:");
  Serial.print(accGyro[0]);
  Serial.print(" Y[mdps]:");
  Serial.print(accGyro[1]);
  Serial.print(" Z[mdps]:");
  Serial.println(accGyro[2]);

  Serial.println("");
}
