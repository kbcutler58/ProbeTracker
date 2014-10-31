
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

#define DEBUG
#ifdef DEBUG
#include "DebugUtils.h"
#endif

#include "CommunicationUtils.h"
#include "FreeSixIMU.h"
#include <Wire.h>

float q[4];
byte SCLK = 2;
byte SDIO = 3;
byte deltax = 0;
byte deltay = 0;
byte squal = 0;

FreeSixIMU my3IMU = FreeSixIMU();

void setup() {
  Serial.begin(115200);
  Wire.begin();
  writeReg(0x00,B00000001);
  delay(5);
  my3IMU.init();
  delay(5);
}

void loop() {
//  writeReg(0x00,B00000001);
  deltax = readReg(0x03);
  deltay = readReg(0x02);
  squal = readReg(0x04);
  my3IMU.getQ(q);
  Serial.print(q[0]);
  Serial.print(",");
  Serial.print(q[1]);
  Serial.print(",");
  Serial.print(q[2]);
  Serial.print(",");
  Serial.print(q[3]);
  Serial.print(",");
//  Serial.print(" X motion: ");
  Serial.print(deltax);
  Serial.print(",");
//  Serial.print(", Y motion: ");
  Serial.print(deltay);
  Serial.print(",");
  Serial.print(squal);
  Serial.println("");
//  writeReg(0x1A,B00000001);
//  writeReg(0x40,B00100001);
//  writeReg(B10101010,B01010101);
//  Serial.println(value);
  delay(10);
}

void writeReg( byte address, byte data) {
  int i = 7;
  address |= 0x80;
  pinMode (SCLK,OUTPUT);
  pinMode (SDIO,OUTPUT);
  //Write Address
  for (; i>=0; i--)  {
    digitalWrite (SCLK,LOW);
    digitalWrite (SDIO, address & (1 << i));
    digitalWrite (SCLK,HIGH);
  }
  //Write data
  for (i=7; i>=0; i--)  {
    digitalWrite(SCLK,LOW);
    digitalWrite(SDIO, data & (1 << i));
    digitalWrite(SCLK,HIGH);
  }
  delayMicroseconds(200);
}

byte readReg( byte address) {
  int i = 7;
  byte value = 0;
  pinMode (SCLK, OUTPUT);
  pinMode (SDIO, OUTPUT);
  for (; i>=0; i--) {
    digitalWrite(SCLK, LOW);
    digitalWrite(SDIO, address & (1 << i));
    digitalWrite(SCLK, HIGH);
  }  
  pinMode(SDIO,INPUT);
  delayMicroseconds(10);
  
  for (i=7; i>=0; i--) {
    digitalWrite(SCLK, LOW);
    digitalWrite(SCLK, HIGH);
    value |= (digitalRead(SDIO) << i);
  }
  delayMicroseconds(100);
  return value;
}
