// Embedded Motion Tracking for Arduino
// Based off Razor AHRS Firmware v1.4.2
// Beckman Laser Institute: Diffuse Optical Spectroscopy Lab
// Created by Kyle Cutler 04/03/14

/*
Modified by Matt Bovyn 7/24/14

Now includes laser controller code to run the cw system built by Soroush
on the same arduino board

laser controller code originally written by Soroush
Matt modified some variable names and addeded some comments
*/

#include <Wire.h> //Orientation Comm Protocol
#include <SPI.h> //Displacement Comm Protocol
#include <avr/pgmspace.h>

//definitions
#define Output_Interval 10
#define output_baud_rate 115200

#define gravity 256.0f
#define kp_Rollpitch 0.02f
#define ki_Rollpitch 0.00002f
#define kp_Yaw 5.2f
#define ki_Yaw 0.00002f

// Calibration values
#define accel_x_min ((float) -250)
#define accel_y_min ((float) -250)
#define accel_z_min ((float) -250)
#define accel_x_max ((float) 250)
#define accel_y_max ((float) 250)
#define accel_z_max ((float) 250)

#define magnet_x_min ((float) -600)
#define magnet_y_min ((float) -600)
#define magnet_z_min ((float) -600)
#define magnet_x_max ((float) 600)
#define magnet_y_max ((float) 600)
#define magnet_z_max ((float) 600)

#define gyro_offset_x ((float) 0.0)
#define gyro_offset_y ((float) 0.0)
#define gyro_offset_z ((float) 0.0)

// Offset and scale calculations (for calibration)
#define accel_x_off ((accel_x_min + accel_x_max) / 2.0f)
#define accel_y_off ((accel_y_min + accel_y_max) / 2.0f)
#define accel_z_off ((accel_z_min + accel_z_max) / 2.0f)
#define accel_x_scale (gravity / (accel_x_max - accel_x_off))
#define accel_y_scale (gravity / (accel_y_max - accel_y_off))
#define accel_z_scale (gravity / (accel_z_max - accel_z_off))
#define magnet_x_off ((magnet_x_min + magnet_x_max) / 2.0f)
#define magnet_y_off ((magnet_y_min + magnet_y_max) / 2.0f)
#define magnet_z_off ((magnet_z_min + magnet_z_max) / 2.0f)
#define magnet_x_scale (100.0f / (magnet_x_max - magnet_x_off))
#define magnet_y_scale (100.0f / (magnet_y_max - magnet_y_off))
#define magnet_z_scale (100.0f / (magnet_z_max - magnet_z_off))

// Gain for gyroscope (ITG-3200)
#define gyro_gain 0.06957 // Same gain on all axes
#define gyro_scaled_rad(x) (x * (gyro_gain * .01745329252)) // Calculate the scaled gyro readings in radians per second

#define to_deg(x) (x * 57.2957795131)
#define to_reg(x) (x * 0.01745329252)

// Orientation Sensor variables
float accel[3], accel_min[3], accel_max[3]; //accelerometer variables
float magnet[3], magnet_min[3], magnet_max[3], magnet_temp[3]; //magnetometer variables
float gyro[3], gyro_average[3]; //gyroscope variables
int gyro_num_samples = 0;
float yaw,pitch,roll; // Euler Angles

// DCM Variables
float mag_heading; //compass heading
float accel_vector[3] = {0,0,0}; //accel data vector
float gyro_vector[3] = {0,0,0}; //gyro data vector
float omega_vector[3] = {0,0,0}; //corrected gyro vector
float omega_P[3] = {0,0,0};
float omega_I[3] = {0,0,0};
float omega[3] = {0,0,0};
float errorRollPitch[3] = {0,0,0};
float errorYaw[3] = {0,0,0};
float DCM_Matrix[3][3],New_Matrix[3][3],Temp_Matrix[3][3];

unsigned long timestamp; //timestamp for main loop
unsigned long timestamp_old; //previous timestamp for main loop
float int_dt; // integration time

// Displacement Sensor variables
int x,y,x_convert,y_convert,x_int,y_int,squal; // raw and translated data
float current_time; // timing for sensor
byte laseroff = 0; // flag for turning off laser for measurements

char OutputMode = 0; // flag for changing raw vs integrative output
// Button control variable
int button_value;

//Variables for laser controller
int laser1_power=2500;
int laser2_power=2500;
int laser_selection=3; //0 1 2 or 3, see DAC_mux
int LD1[100]={}; //variable to store data at first frequency
int LD2[100]={}; //variable to store data at second frequency
char tStr1[500]; //string to output data at first frequency
char tStr2[500]; //string to output data at second frequency
int A=6; //first pin for choosing which diode to send RF modulation to
int B=7; 
int MUX_EN=8;
byte start_flag=0; //added by Matt


void setup() {
// Initialization functions

Serial.begin(output_baud_rate);
delay(50);

// I2C Comm, Sensors: ADXL345, HMC5883L, ITG-3200, ADNS9800
I2C_Init();
gyro_Init();
magnet_Init();
accel_Init();

// SPI Comm, Sensor: ADNS9800
SPI_Init();
lasermouse_Init(); //includes uploading firmware
delay(20);
reset_fusion(); //done
delay(200);

//Laser controller Variables
ADC->ADC_MR |= 0x80; // these lines set free running mode on adc 7 (pin A0)
ADC->ADC_CR=2;
ADC->ADC_CHER=0xC0; // this is (1<<7) | (1<<6) for adc 7 and adc 6  
pinMode(MUX_EN,OUTPUT);
}

void loop() {

if (start_flag==1) //wait to start the system until we send it an 'a'
{
if((millis() - timestamp) >= Output_Interval)
{
  timestamp_old = timestamp;
  timestamp = millis();
  if (timestamp > timestamp_old)
  int_dt = (float) (timestamp - timestamp_old) / 1000.0f;
  else int_dt = 0;
  
  //turn on laser diodes
  DAC_mux(laser_selection,laser1_power,laser2_power);
  
  //get tracking stuff  
  read_accel();
  read_gyro();
  read_magnet();
  update_button();
  if (laseroff == 0) read_lasermouse();
  
  //do tracking math
  compensate_errors(); //scale and offset
  compass_heading(); //get a magnetic heading
  matrix_update(); // updates the DCM matrix
  normalize_values(); // normalize DCM
  drift_correction(); // check and correct for drift
  convert_angles(); // from matrix to euler
  
  //output data from optical signals
//    for (int i=0; i<num; i++) //read data and save to LD1 and LD2 variables
//    {
//    //while((ADC->ADC_ISR & 0x80)==0); // wait for conversion 1channel
//    while((ADC->ADC_ISR & 0xC0)!=0xC0); // 2channel
//    LD1[i]=ADC->ADC_CDR[A]; // read data
//    LD2[i]=ADC->ADC_CDR[B];
//    }
  
  //turn of lasers (just in case)
  //DAC_mux(0,0,0)
  
  //Convert LD1 and LD2 to strings (for speed - can't send array all at once)
  snprintf(tStr1,sizeof(tStr1),"%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%u",LD1[0],LD1[1],LD1[2],LD1[3],LD1[4],LD1[5],LD1[6],LD1[7],LD1[8],LD1[9], LD1[10], LD1[11],LD1[12],LD1[13],LD1[14],LD1[15],LD1[16],LD1[17],LD1[18],LD1[19], LD1[20], LD1[21],LD1[22],LD1[23],LD1[24],LD1[25],LD1[26],LD1[27],LD1[28],LD1[29], LD1[30], LD1[31],LD1[32],LD1[33],LD1[34],LD1[35],LD1[36],LD1[37],LD1[38],LD1[39], LD1[40], LD1[41],LD1[42],LD1[43],LD1[44],LD1[45],LD1[46],LD1[47],LD1[48],LD1[49], LD1[50], LD1[51],LD1[52],LD1[53],LD1[54],LD1[55],LD1[56],LD1[57],LD1[58],LD1[59], LD1[60], LD1[61],LD1[62],LD1[63],LD1[64],LD1[65],LD1[66],LD1[67],LD1[68],LD1[69], LD1[70], LD1[71],LD1[72],LD1[73],LD1[74],LD1[75],LD1[76],LD1[77],LD1[78],LD1[79], LD1[80], LD1[81],LD1[82],LD1[83],LD1[84],LD1[85],LD1[86],LD1[87],LD1[88],LD1[89], LD1[90], LD1[91],LD1[92],LD1[93],LD1[94],LD1[95],LD1[96],LD1[97],LD1[98],LD1[99]);
  snprintf(tStr2,sizeof(tStr2),"%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%uH%u",LD2[0],LD2[1],LD2[2],LD2[3],LD2[4],LD2[5],LD2[6],LD2[7],LD2[8],LD2[9], LD2[10], LD2[11],LD2[12],LD2[13],LD2[14],LD2[15],LD2[16],LD2[17],LD2[18],LD2[19], LD2[20], LD2[21],LD2[22],LD2[23],LD2[24],LD2[25],LD2[26],LD2[27],LD2[28],LD2[29], LD2[30], LD2[31],LD2[32],LD2[33],LD2[34],LD2[35],LD2[36],LD2[37],LD2[38],LD2[39], LD2[40], LD2[41],LD2[42],LD2[43],LD2[44],LD2[45],LD2[46],LD2[47],LD2[48],LD2[49], LD2[50], LD2[51],LD2[52],LD2[53],LD2[54],LD2[55],LD2[56],LD2[57],LD2[58],LD2[59], LD2[60], LD2[61],LD2[62],LD2[63],LD2[64],LD2[65],LD2[66],LD2[67],LD2[68],LD2[69], LD2[70], LD2[71],LD2[72],LD2[73],LD2[74],LD2[75],LD2[76],LD2[77],LD2[78],LD2[79], LD2[80], LD2[81],LD2[82],LD2[83],LD2[84],LD2[85],LD2[86],LD2[87],LD2[88],LD2[89], LD2[90], LD2[91],LD2[92],LD2[93],LD2[94],LD2[95],LD2[96],LD2[97],LD2[98],LD2[99]);

  //output results
  output_print(); // print output data to serial
  
}
else {} //if not started, do nothing
}

if (Serial.available() >= 1) {
  if (Serial.read() == 'a') {
    start_flag=1; //send the system an a to start it
  }
  else if (Serial.read()=='b') {
    start_flag=0; //send the system a b to stop it
  }
  else if (Serial.read() == 'r') {
    reset_function();
  }
  else if (Serial.read() == 'm') {
    change_OutputMode();
  }
  else if (Serial.read() == 'h') {
    reset_Integrated();
  }
}

} //end of loop

void output_print() {
  Serial.print(millis());
  Serial.print(",");
  if (OutputMode == 'integrated') {
    Serial.print(x_int);
    Serial.print(",");
    Serial.print(y_int);
  }
  else {
  Serial.print(x_convert);
  Serial.print(",");
  Serial.print(y_convert);
  }
  Serial.print(",");
  Serial.print(to_deg(yaw));
  Serial.print(",");
  Serial.print(to_deg(pitch));
  Serial.print(",");
  Serial.print(to_deg(roll));
  Serial.print(",");
  Serial.print(button_value);
  Serial.print(",");
  Serial.print(squal);
//  Serial.print(",");
//  Serial.print(tStr1);
//  Serial.print(",");
//  Serial.print(tStr2);
  Serial.println(" ");

}

void update_button() {
  button_value=analogRead(0);
}

void reset_function() {
  pinMode(9,OUTPUT);
  digitalWrite(9,LOW);
}

void change_OutputMode() {
  if (OutputMode == 'raw') { OutputMode = 'integrated'; }
  if (OutputMode == 'integrated') { OutputMode = 'raw'; }
}


void reset_Integrated() {
  x_int = 0;
  y_int = 0;
}

//laser controller function
void DAC_mux(int LD, int pow1, int pow2)
{
pinMode(A,OUTPUT); //pin for the first laser diode, pin number set above
pinMode(B,OUTPUT); //pin for second diode, set above
digitalWrite(MUX_EN,LOW); 
  
   switch (LD) //0, 1, 2, or 3 sent to the function
  {
    case 0: //turn on neither diode
      digitalWrite(A,LOW);
      digitalWrite(B,LOW);
      break;
    case 1: //turn on first diode only
      digitalWrite(A,HIGH);
      digitalWrite(B,LOW);
      break;
    case 2: //turn on second diode only
      digitalWrite(A,LOW);
      digitalWrite(B,HIGH);
      break;
    case 3: //turn on both diodes
      digitalWrite(A,HIGH);
      digitalWrite(B,HIGH);
      break;
    default: //do nothing if something else sent
      break;
  }//end of MUX selection
  
  analogWrite(DAC0,pow1); //set modulation amplitude for laser diode 1?
  analogWrite(DAC1,pow2); //set modulation amplitude for laser diode 2?
  
} //end of DAC_mux function
