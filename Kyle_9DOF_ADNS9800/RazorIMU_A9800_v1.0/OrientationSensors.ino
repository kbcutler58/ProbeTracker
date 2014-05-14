#define accel_address ((int) 0x53)
#define magnet_address ((int) 0x1E)
#define gyro_address ((int) 0x68)

void I2C_Init()
{
  Wire.begin();
}

void accel_Init()
{
  // Set to measurement mode
  Wire.beginTransmission(accel_address);
  Wire.write(0x2D); // Power Control Register 
  Wire.write(0x08); // Starts measurment mode on B00001000
  Wire.endTransmission();
  delay(10);
  
  // Set resolution
  Wire.beginTransmission(accel_address);
  Wire.write(0x31); // Data Format Register
  Wire.write(0x08); // Put device in full resolution B00001000
  Wire.endTransmission();
  delay(10);
  
  // Set sampling
  Wire.beginTransmission(accel_address);
  Wire.write(0x2C); // Rate Control Register
    // Output Rates Hz    3200, 1600, 0800, 0400, 0200, 0100, 0050, 0025, 0012.5, 0006.25
    // Rate Codes B0-B3   1111, 1110, 1101, 1100, 1011, 1010, 1001, 1000, 0111,   0110 
  Wire.write(0x09); // Turn off lower power mode set to 50hz data B0000 Rate Code 1001
  Wire.endTransmission();
  delay(10);
}
  
void gyro_Init()
{
  // Reset Power Settings 
  Wire.beginTransmission(gyro_address);
  Wire.write(0x3E); // Power Management Register
  Wire.write(0x80); // Reset power settings B10000000
  Wire.endTransmission();
  delay(10);
  
  // Set Resolution and Low Pass Filter
  Wire.beginTransmission(gyro_address);
  Wire.write(0x16); // Filter and Resolution Register
  Wire.write(0x1B); // Set full resolution 03h Low Pass Bandwitdh 42Hz (256Hz to 5Hz)
  Wire.endTransmission();
  delay(10);
  
  // Set Sampling Rate
  Wire.beginTransmission(gyro_address);
  Wire.write(0x15); // Sample Rate Divider
    // Fsample = Finternal(1khz) / (divider+1) = 1khz to 1000/
  Wire.write(0x0A); // Divider = 10 (50Hz)   
  Wire.endTransmission();
  delay(10);
  
  // Set Clock to z gyro reference
  Wire.beginTransmission(gyro_address);
  Wire.write(0x35); // Power Management Register
  Wire.write(0x03); // PLL with Z Gyro Reference
  // Alernate Wire.write(0x00) Set Clock Select to internal oscillator
  Wire.endTransmission();
  delay(10);  
}

void magnet_Init()
{
  // Set Measurement Mode 
  Wire.beginTransmission(magnet_address);
  Wire.write(0x02); // Mode Register
  Wire.write(0x00); // Set to continuous-measurement mode
  Wire.endTransmission();
  delay(10);
  
  // Set Data Output Rate and Measurement Config
  Wire.beginTransmission(magnet_address);
  Wire.write(0x00); // Configuration Register A
  Wire.write(0x18); // Data Output Rate Bits B4-B2 000 = .75hz 110 = 75
  Wire.endTransmission();
  delay(10);
}

void read_accel()
{
  int i = 0;
  uint8_t accelbuff[6]; // Read 6 bytes from accel
  
  Wire.beginTransmission(accel_address);
  Wire.write(0x32); // Address to start read
  Wire.endTransmission();
  
  Wire.beginTransmission(accel_address);
  Wire.requestFrom(accel_address, 6); // Request 6 bytes
  while(Wire.available())  // ((Wire.available()) & i<6
  {
    accelbuff[i] = Wire.read();
    i++;
  }
  Wire.endTransmission();
  if (i==6) // If all bytes 
  {
    accel[0] = (int16_t)(((uint16_t) accelbuff[3]) << 8) | accelbuff[2]; // X axis (internal Y)
    accel[1] = (int16_t)(((uint16_t) accelbuff[1]) << 8) | accelbuff[0]; // Y axis (internal X) 
    accel[2] = (int16_t)(((uint16_t) accelbuff[5]) << 8) | accelbuff[4]; // Z axis (internal Z)
  }
  else
  {
    Serial.println("Error reading accel");
  }
}

void read_gyro()
{
  int i = 0;
  uint8_t gyrobuff[6];
  
  Wire.beginTransmission(gyro_address);
  Wire.write(0x1D); // Address to start read
  Wire.endTransmission();

  Wire.beginTransmission(gyro_address);
  Wire.requestFrom(gyro_address, 6); // Request 6 bytes
  while(Wire.available())
  {
  gyrobuff[i] = Wire.read();  // Read one byte at a time
  i++;
  }
  Wire.endTransmission();
  
  if (i==6) // If all bytes in
  {
    gyro[0] = -1*(int16_t)((((uint16_t) gyrobuff[2]) << 8) | gyrobuff[3]);  // X axis(internal -Y)
    gyro[1] = -1*(int16_t)((((uint16_t) gyrobuff[0]) << 8) | gyrobuff[1]);  // Y axis(internal -X)
    gyro[2] = -1*(int16_t)((((uint16_t) gyrobuff[4]) << 8) | gyrobuff[5]);  // Z axis(internal -Z)
  }
  else
  {
    Serial.println("Error reading gyro");
  }
}

void read_magnet()
{
  int i = 0;
  uint8_t magnetbuff[6];
  
  Wire.beginTransmission(magnet_address);
  Wire.write(0x03);
  Wire.endTransmission();
  
  Wire.beginTransmission(magnet_address);
  Wire.requestFrom(magnet_address, 6);
  while(Wire.available())
  {
    magnetbuff[i] = Wire.read();
    i++;
  }
  Wire.endTransmission();
  
  if (i==6)
  {
    magnet[0] = (int16_t)((((uint16_t) magnetbuff[0]) << 8) | magnetbuff[1]);     // X axis (internal X)
    magnet[1] = -1*(int16_t)((((uint16_t) magnetbuff[4]) << 8) | magnetbuff[5]); // Y axis (internal -Y)
    magnet[2] = -1*(int16_t)((((uint16_t) magnetbuff[2]) << 8) | magnetbuff[3]); // Z Axis (internal -Z)
  }
  else
  {
    Serial.println("Error Reading Magnetometer");
  }
}

void compass_heading()
{
  float magnet_x;
  float magnet_y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
  
  cos_roll = cos(roll);
  sin_roll = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);
  
  // Tilt compensated magnetic field X
  magnet_x = magnet[0] * cos_pitch + magnet[1] * sin_roll * sin_pitch + magnet[2] * cos_roll * sin_pitch;
  // Tilt compensated magnetic field Y
  magnet_y = magnet[1] * cos_roll - magnet[2] * sin_roll;
  // Magnetic Heading
  mag_heading = atan2(-magnet_y, magnet_x);
}
