// This is code to read the A2610 Avago Optical Mouse Sensor
// Kyle Cutler 3/6/14

byte SCLK = 2;
byte SDIO = 3;
byte x,y,squal;

void A2610_Init()
{
  // 0x00 is operational register
  // 0x01 turns always on
  A2610_write_reg(0x00,B00000001);
  delay(5);
  //Write always on to chip  
}

void A2610_read_sensor()
{
  x = A2610_read_reg(0x03);
  y = A2610_read_reg(0x02);
  squal = A2610_read_reg(0x04);
 //Read xy data
 //read x value at register address
 //store x value
 //read y value at register address
 //store y value
 //read squal at register address
 //store squal value
}

void A2610_output()
{
  //Print output variables to serial
  Serial.print(x); Serial.print(",");
  Serial.print(y); Serial.print(",");
  Serial.print(squal); Serial.print(",");
   
}


void A2610_write_reg( byte address, byte data)
{
  //basic digital write
  int i = 7;
  address |= 0x80;
  pinMode(SCLK,OUTPUT);
  pinMode(SDIO,OUTPUT);
  //Write Address
  for (; i>=0; i--)  {
    digitalWrite(SCLK,LOW);
    digitalWrite(SDIO, address & (1 << i));
    digitalWrite(SCLK,HIGH);
  }
  //Write data
  for (i=7; i>=0; i--)  {
    digitalWrite(SCLK,LOW);
    digitalWrite(SDIO, data & (1 << i));
    digitalWrite(SCLK,HIGH);
  }
  delayMicroseconds(200);
}

byte A2610_read_reg( byte address )
{
  int i = 7;
  byte value = 0;
  pinMode(SCLK, OUTPUT);
  pinMode(SDIO, OUTPUT);
  for (; i>=0; i--) {
    digitalWrite(SCLK, LOW);
    digitalWrite(SDIO, address & (1 << i));
    digitalWrite(SCLK, HIGH);
  }
  pinMode(SDIO, INPUT);
  delayMicroseconds(10);
  
  for (i=7; i>=0; i--) {
    digitalWrite(SCLK, LOW);
    digitalWrite(SCLK, HIGH);
    value |= (digitalRead(SDIO) << i);
  }
  delayMicroseconds(100);
  return value;
}
