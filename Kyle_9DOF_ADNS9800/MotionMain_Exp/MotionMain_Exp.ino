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
float accel[3];
float magnet[3];
float gyro[3];
int x,y,x_convert,y_convert;

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

reset_Fusion();
}

void loop() {
read_accel();
read_gyro();
read_magnet();
//Compensate sensor errors

//Calculate magnetic heading

//Update Matrix

//Normalize

//Drift Correction

//Euler Angles

read_lasermouse();
Serial.print("x= ");
Serial.print(x_convert);
Serial.print(", y= ");
Serial.println(y_convert);
// Serial.println(magnet[0]);
// Serial.println(gyro[0]);
// Serial.println(accel[0]);
 delay(50);


//read_orient_sensors();
//read_displacement_sensor();
//output_Print();
}
