// Embedded Motion Tracking for Arduino
// Based off Razor AHRS Firmware v1.4.2
// Beckman Laser Institute: Diffuse Optical Spectroscopy Lab
// Created by Kyle Cutler 04/03/14

/*
Version 2.2 Made specifically for arduino uno
 Code included to read lasers but commented out
 */

#include <Wire.h> //Orientation Comm Protocol
#include <SPI.h> //Displacement Comm Protocol
#include <avr/pgmspace.h>

//definitions
#define Output_Interval 20 // 50hz  
#define output_baud_rate 115200
#define gravity 256.0f
#define kp_Rollpitch 0.02f
#define ki_Rollpitch 0.00002f
#define kp_Yaw 1.2f
#define ki_Yaw 0.00002f

//// Calibration values
//#define accel_x_min ((float) -250)
//#define accel_y_min ((float) -250)
//#define accel_z_min ((float) -250)
//#define accel_x_max ((float) 250)
//#define accel_y_max ((float) 250)
//#define accel_z_max ((float) 250)
//
//#define magnet_x_min ((float) -600)
//#define magnet_y_min ((float) -600)
//#define magnet_z_min ((float) -600)
//#define magnet_x_max ((float) 600)
//#define magnet_y_max ((float) 600)
//#define magnet_z_max ((float) 600)
//
//#define gyro_offset_x ((float) 0.0)
//#define gyro_offset_y ((float) 0.0)
//#define gyro_offset_z ((float) 0.0)


// Calibration values
#define accel_x_min ((float) -313)
#define accel_y_min ((float) -275)
#define accel_z_min ((float) -264)
#define accel_x_max ((float) 268)
#define accel_y_max ((float) 272)
#define accel_z_max ((float) 417)

#define magnet_x_min ((float) -600)
#define magnet_y_min ((float) -600)
#define magnet_z_min ((float) -600)
#define magnet_x_max ((float) 600)
#define magnet_y_max ((float) 600)
#define magnet_z_max ((float) 600)

#define gyro_offset_x ((float) 0)
#define gyro_offset_y ((float) 0)
#define gyro_offset_z ((float) 0)

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

byte use_calibration = 0;

// Orientation Sensor variables
float accel[3], accel_min[3], accel_max[3]; //accelerometer variables
float magnet[3], magnet_min[3], magnet_max[3], magnet_temp[3]; //magnetometer variables
float gyro[3], gyro_average[3]; //gyroscope variables
int gyro_num_samples = 0;
float yaw, pitch, roll; // Euler Angles

// DCM Variables
float mag_heading; //compass heading
float accel_vector[3] = {
  0, 0, 0}; //accel data vector
float gyro_vector[3] = {
  0, 0, 0}; //gyro data vector
float omega_vector[3] = {
  0, 0, 0}; //corrected gyro vector
float omega_P[3] = {
  0, 0, 0};
float omega_I[3] = {
  0, 0, 0};
float omega[3] = {
  0, 0, 0};
float errorRollPitch[3] = {
  0, 0, 0};
float errorYaw[3] = {
  0, 0, 0};
float DCM_Matrix[3][3], New_Matrix[3][3], Temp_Matrix[3][3];

unsigned long timestamp; //timestamp for main loop
unsigned long timestamp_old; //previous timestamp for main loop
float int_dt; // integration time

// Orientation Calibration Variables
int curr_calibration_sensor = 0;

// Displacement Sensor variables
int x_low, x_high, y_low, y_high, x_int, y_int, squal; // raw and translated data
float current_time; // timing for sensor
byte laseroff = 0; // flag for turning off laser for measurements

// Button control variable
int button_value;

byte start_flag = 0; //added by Matt

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

  Serial.println("Setup Complete. Send a to continue"); //let user know
}

void loop() {

  //  digitalWrite(MUX_EN,HIGH );
  if (Serial.available() >= 1) {
    if (Serial.read() == 'a') {
      start_flag = 1; //send the system an a to start it
    }
    else if (Serial.read() == 'b') {
      start_flag = 0; //send the system a b to stop it
    }
    else if (Serial.read() == 'r') {
      reset_function();
    }
    else {
    }
  }

  if (start_flag == 1) //wait to start the system until we send it an 'a'
  {
    if ((millis() - timestamp) >= Output_Interval)
    {
      timestamp_old = timestamp;
      timestamp = millis();
      if (timestamp > timestamp_old) {
        int_dt = (float) (timestamp - timestamp_old) / 1000.0f;
      }
      else {
        int_dt = 0;
      }

      read_accel();
      read_gyro();
      read_magnet();
      update_button();
      if (laseroff == 0)
      {
        read_lasermouse();
      }
      if (use_calibration == 1)
      {
        for (int i = 0; i < 3; i++) {
          output_calibration(curr_calibration_sensor);
          curr_calibration_sensor++;
        }
        curr_calibration_sensor = 0;
      }
      //do tracking math
      compensate_errors(); //scale and offset
      compass_heading(); //get a magnetic heading
      matrix_update(); // updates the DCM matrix
      normalize_values(); // normalize DCM
      drift_correction(); // check and correct for drift
      convert_angles(); // from matrix to euler
      //  Serial.print(millis());

      //output results
      output_print(); // print output data to Serial
    }
  }

} //end of loop

void output_print() {
  Serial.print(millis());
  Serial.print(",");
  Serial.print(x_low);
  Serial.print(",");
  Serial.print(x_high);
  Serial.print(",");
  Serial.print(y_low);
  Serial.print(",");
  Serial.print(y_high);
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
  button_value = analogRead(0);
}

void reset_function() {
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);
}


