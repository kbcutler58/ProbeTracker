// Embedded Motion Tracking for Arduino
// Based off Razor AHRS Firmware v1.4.2
// Beckman Laser Institute: Diffuse Optical Spectroscopy Lab
// Created by Kyle Cutler 04/03/14

#include <Wire.h> //Orientation Comm Protocol
#include <SPI.h> //Displacement Comm Protocol
#include <avr/pgmspace.h>

//definitions
#define Output_Interval 25
#define output_baud_rate 57600

#define gravity 256.0f
#define kp_Rollpitch 0.02f
#define ki_Rollpitch 0.00002f
#define kp_Yaw 1.2f
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
float gyro_vector[3] = {0,0,0};  //gyro data vector
float omega_vector[3] = {0,0,0}; //corrected gyro vector
float omega_P[3] = {0,0,0};
float omega_I[3] = {0,0,0};
float omega[3] = {0,0,0};
float errorRollPitch[3] = {0,0,0};
float errorYaw[3] = {0,0,0};
float DCM_Matrix[3][3],New_Matrix[3][3],Temp_Matrix[3][3];

unsigned long timestamp; //timestamp for main loop
unsigned long timestamp_old; //previous timestamp for main loop
float int_dt;  // integration time

// Displacement Sensor variables
int x,y,x_convert,y_convert,x_int,y_int,squal; // raw and translated data
float current_time; // timing for sensor
byte laseroff = 0; // flag for turning off laser for measurements

char OutputMode = 0; // flag for changing raw vs integrative output
// Button control variable
int button_value;

void setup() {
// Initialization functions

Serial.begin(output_baud_rate);
delay(50);

// I2C Comm,  Sensors: ADXL345, HMC5883L, ITG-3200, ADNS9800
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
}

void loop() {
  
if((millis() - timestamp) >= Output_Interval)
{
  timestamp_old = timestamp;
  timestamp = millis();
  if (timestamp > timestamp_old)
  int_dt = (float) (timestamp - timestamp_old) / 1000.0f;
  else int_dt = 0;
  
  read_accel();
  read_gyro();
  read_magnet();
  update_button();
  if (laseroff == 0) read_lasermouse();
  
  compensate_errors(); //scale and offset
  compass_heading(); //get a magnetic heading
  matrix_update(); // updates the DCM matrix
  normalize_values(); // normalize DCM
  drift_correction(); // check and correct for drift
  convert_angles(); // from matrix to euler
  output_print(); // print output data to serial
}

if (Serial.available() >= 1) {
  if (Serial.read() == 'r') {
    reset_function();
  }
  else if (Serial.read() == 'm') {
    change_OutputMode();
  }
  else if (Serial.read() == 'h') {
    reset_Integrated();
  }
}

}
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
  Serial.println(" ");

}

void update_button() {
  button_value=analogRead(0);
}

void reset_function() {
  pinMode(7,OUTPUT);
  digitalWrite(7,LOW);
}  

void change_OutputMode() {
  if (OutputMode == 'raw') { OutputMode = 'integrated'; }
  if (OutputMode == 'integrated') { OutputMode = 'raw'; }  
}


void reset_Integrated() {
  x_int = 0;
  y_int = 0;
}
 
