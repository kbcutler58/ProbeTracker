// Embedded Motion Tracking for Arduino
// Based off Razor AHRS Firmware v1.4.2
// Beckman Laser Institute: Diffuse Optical Spectroscopy Lab
// Created by Kyle Cutler 04/03/14


//definitions
#define output_baud_rate 57600
#include <Wire.h> //Orientation Comm Protocol
#include <SPI.h> //Displacement Comm Protocol
#include <avr/pgmspace.h>

//declarations
float accel[3],magnet[3],gyro[3],yaw,pitch,roll;
int x,y,x_convert,y_convert;
float time;
byte laseroff = 0;
float DCM_Matrix[3][3],New_Matrix[3][3],Temp_Matrix[3][3];
float accel_x_off,accel_y_off,accel_z_off,accel_x_scale,accel_y_scale,accel_z_scale;
float gyro_offset_x,gyro_offset_y,gyro_offset_z;
float magnet_x_off,magnet_y_off,magnet_z_off,magnet_x_scale,magnet_y_scale,magnet_z_scale;
float mag_heading;


void output_Print()
{
 Serial.print("x= ");
 Serial.print(x_convert);
 Serial.print(", y= ");
 Serial.println(y_convert);
 Serial.println(magnet[0]);
 Serial.println(gyro[0]);
 Serial.println(accel[0]);
 Serial.print(time);
}

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

// Initial Calibration: Read Sensors, initalize DCM
//orientation_Calib();
//displacement_Calib();

reset_fusion(); //done
}

void loop() {
time = millis();
read_accel();
read_gyro();
read_magnet();
if (laseroff == 0) read_lasermouse();

compensate_errors(); //scale and offset
compass_heading(); //get a magnetic heading
matrix_update(); // updates the DCM matrix
normalize_values(); // normalize DCM
drift_correction(); // check and correct for drift
convert_angles(); // from matrix to euler

output_Print(); // print output data to serial
}
