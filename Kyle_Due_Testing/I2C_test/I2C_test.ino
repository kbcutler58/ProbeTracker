#include <Wire.h>

float accel[3];
float gyro[3];
#define accel_address 0x53
#define accel_power_register 0x2d
#define accel_data_register 0x32

#define gyro_address 0x68


void setup() {
  Serial.begin(57600);
  Wire.begin();
  delay(10);
  accel_init();
//  gyro_init();
  delay(10);
}

void loop() {
  read_sensors();
  Serial.print(accel[0]);
  Serial.print(",");
  Serial.print(accel[1]);
  Serial.print(",");
  Serial.print(accel[2]);
  Serial.println("");
  delay(100);
}

void accel_init()
{
  //send power on to accel power register
  Wire.beginTransmission(accel_address);
  Wire.write(accel_power_register);
  Wire.write(8);  
  Wire.endTransmission();
}

//void gyro_init()
//{
////  Wire.beginTransmission(gyro_address);
//}

void read_sensors()
{
  int i = 0;
  uint8_t accelbuff[6];
  Wire.beginTransmission(accel_address);
  Wire.write(accel_data_register);
  Wire.endTransmission();
  
  Wire.beginTransmission(accel_address);
  Wire.requestFrom(accel_address, 6);
  while(Wire.available())
  {
    accelbuff[i] = Wire.read();
    i++;
  }
  Wire.endTransmission();
    if (i==6) // If all bytes 
  {
    accel[0] = (int16_t)(((uint16_t) accelbuff[1]) << 8) | accelbuff[0]; // X axis (internal Y)
    accel[1] = (int16_t)(((uint16_t) accelbuff[3]) << 8) | accelbuff[2]; // Y axis (internal X) 
    accel[2] = (int16_t)(((uint16_t) accelbuff[5]) << 8) | accelbuff[4]; // Z axis (internal Z)
  }
  else
  {
    Serial.println("Error reading accel");
  }
}


