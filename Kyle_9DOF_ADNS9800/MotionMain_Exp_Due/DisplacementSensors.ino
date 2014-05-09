#define SS_PIN 10
#define REG_Product_ID                           0x00
#define REG_Revision_ID                          0x01
#define REG_Motion                               0x02
#define REG_Delta_X_L                            0x03
#define REG_Delta_X_H                            0x04
#define REG_Delta_Y_L                            0x05
#define REG_Delta_Y_H                            0x06
#define REG_SQUAL                                0x07
#define REG_Pixel_Sum                            0x08
#define REG_Maximum_Pixel                        0x09
#define REG_Minimum_Pixel                        0x0a
#define REG_Shutter_Lower                        0x0b
#define REG_Shutter_Upper                        0x0c
#define REG_Frame_Period_Lower                   0x0d
#define REG_Frame_Period_Upper                   0x0e
#define REG_Configuration_I                      0x0f
#define REG_Configuration_II                     0x10
#define REG_Frame_Capture                        0x12
#define REG_SROM_Enable                          0x13
#define REG_Run_Downshift                        0x14
#define REG_Rest1_Rate                           0x15
#define REG_Rest1_Downshift                      0x16
#define REG_Rest2_Rate                           0x17
#define REG_Rest2_Downshift                      0x18
#define REG_Rest3_Rate                           0x19
#define REG_Frame_Period_Max_Bound_Lower         0x1a
#define REG_Frame_Period_Max_Bound_Upper         0x1b
#define REG_Frame_Period_Min_Bound_Lower         0x1c
#define REG_Frame_Period_Min_Bound_Upper         0x1d
#define REG_Shutter_Max_Bound_Lower              0x1e
#define REG_Shutter_Max_Bound_Upper              0x1f
#define REG_LASER_CTRL0                          0x20
#define REG_Observation                          0x24
#define REG_Data_Out_Lower                       0x25
#define REG_Data_Out_Upper                       0x26
#define REG_SROM_ID                              0x2a
#define REG_Lift_Detection_Thr                   0x2e
#define REG_Configuration_V                      0x2f
#define REG_Configuration_IV                     0x39
#define REG_Power_Up_Reset                       0x3a
#define REG_Shutdown                             0x3b
#define REG_Inverse_Product_ID                   0x3f
#define REG_Motion_Burst                         0x50
#define REG_SROM_Load_Burst                      0x62
#define REG_Pixel_Burst                          0x64

extern const unsigned short firmware_length;
extern prog_uchar firmware_data[];

void SPI_Init()
{
  pinMode (SS_PIN, OUTPUT);
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(8);
}

void lasermouse_Init()
{
  digitalWrite(SS_PIN, HIGH); //comm end
  digitalWrite(SS_PIN, LOW);  //comm begin
  digitalWrite(SS_PIN, HIGH); //comm end
  write_Register(REG_Power_Up_Reset, 0x5A);
  delay(50);
  upload_Firmware();
  delay(10);
  byte laser_ctrl0 = read_Register(REG_LASER_CTRL0);
  Serial.println(laser_ctrl0);
  write_Register(REG_LASER_CTRL0, laser_ctrl0 & 0xf0);
  delay(10);
  Serial.print("ADNS9800 Initialized");
}

void read_lasermouse()
{
  digitalWrite(SS_PIN, LOW);
  x = read_Register(0x03);
  y = read_Register(0x05);
  digitalWrite(SS_PIN, HIGH);
//  Serial.println("Read Complete");
  x_convert = lasermouse_dataconvert(x);
  y_convert = lasermouse_dataconvert(y);  
}

int lasermouse_dataconvert(int b)
{
  if(b & 0x80) {
    b = -1 * ((b ^ 0xff) +1);
  }
  return b;  
}

// Upload the firmware at boot
void upload_Firmware(){
  Serial.println("Uploading firmware...");
  // set the configuration_IV register in 3k firmware mode
  write_Register(REG_Configuration_IV, 0x02); // bit 1 = 1 for 3k mode, other bits are reserved 
 
  // write 0x1d in SROM_enable reg for initializing
  write_Register(REG_SROM_Enable, 0x1d); 
  
  // wait for more than one frame period
  delay(10); // assume that the frame rate is as low as 100fps... even if it should never be that low
  
  // write 0x18 to SROM_enable to start SROM download
  write_Register(REG_SROM_Enable, 0x18); 
  
  // write the SROM file (=firmware data) 
  digitalWrite(SS_PIN, LOW);
  SPI.transfer(REG_SROM_Load_Burst | 0x80); // write burst destination adress
  delayMicroseconds(15);
  
  // send all bytes of the firmware
  unsigned char c;
  for(int i = 0; i < firmware_length; i++){ 
    c = (unsigned char)pgm_read_byte(firmware_data + i);
    SPI.transfer(c);
    delayMicroseconds(15);
  }
  digitalWrite(SS_PIN, HIGH);
  }

void write_Register(byte reg_addr, byte data)
{
  digitalWrite(SS_PIN, LOW);
  SPI.transfer(reg_addr | 0x80);
  SPI.transfer(data);
  delayMicroseconds(20);
  digitalWrite(SS_PIN, HIGH);
  delayMicroseconds(100);
  
}

byte read_Register(byte reg_addr)
{
  digitalWrite(SS_PIN, LOW);
  SPI.transfer(reg_addr & 0x7F);
  delayMicroseconds(100);
  byte data = SPI.transfer(0);
  delayMicroseconds(1);
  digitalWrite(SS_PIN, HIGH);
  delayMicroseconds(19);
  return data;
}

void laser_off(void)
{
  digitalWrite(ncs,LOW);
  byte laser_ctrl0 = adns_read_reg(REG_LASER_CTRL0);
  adns_write_reg(REG_LASER_CTRL0, laser_ctrl0 | 0x01);
  digitalWrite(ncs,HIGH);
}

void laser_on(void)
{ 
  digitalWrite(ncs,LOW);
  byte laser_ctrl0 = adns_read_reg(REG_LASER_CTRL0);
  adns_write_reg(REG_LASER_CTRL0, laser_ctrl0 & 0xf0);
  digitalWrite(ncs,HIGH);
}
  


