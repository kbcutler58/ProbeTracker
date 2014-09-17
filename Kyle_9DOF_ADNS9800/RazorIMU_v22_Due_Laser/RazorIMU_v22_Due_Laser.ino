/*
 Embedded Motion Tracking for Arduino
 Based off Razor AHRS Firmware v1.4.2
 Beckman Laser Institute: Diffuse Optical Spectroscopy Lab
 Created by Kyle Cutler 04/03/14
 V2.2 8/25/14

Version 2.2 Made specifically for arduino due and CW laser imaging system (Soroush)
Code segmented for options of
  Read orientation data (I2C)
  Read displacement data (SPI)
  Read laser data (Digital Control, Analog Read)
  Force print of all variables
*/

#include <Wire.h> //Orientation Comm Protocol
#include <SPI.h> //Displacement Comm Protocol
#include <avr/pgmspace.h>

//definitions
#define Output_Interval 20
#define output_baud_rate 115200

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

//
//// Calibration values
//#define accel_x_min ((float) -512)
//#define accel_y_min ((float) -512)
//#define accel_z_min ((float) -512)
//#define accel_x_max ((float) 511)
//#define accel_y_max ((float) 365)
//#define accel_z_max ((float) 511)
//
//#define magnet_x_min ((float) -1393)
//#define magnet_y_min ((float) -1308)
//#define magnet_z_min ((float) -1160)
//#define magnet_x_max ((float) 1615)
//#define magnet_y_max ((float) 1643)
//#define magnet_z_max ((float) 4096)
//
//#define gyro_offset_x ((float) 0)
//#define gyro_offset_y ((float) 0)
//#define gyro_offset_z ((float) 0)

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

// Program Options (Change to 0 to turn off)
byte use_orientation = 0; // Will turn on and off i2c and orientation measurements
byte use_displacement = 0; // Will turn on and off SPI and displacement measurements
byte use_lasers = 1; // Will turn on CW laser measurements
byte output_all = 1; // Will output all data even if off

byte use_calibration = 0;

byte use_counter = 1;
int counter = 10;

// Orientation Sensor variables
float accel[3], accel_min[3], accel_max[3]; //accelerometer variables
float magnet[3], magnet_min[3], magnet_max[3], magnet_temp[3]; //magnetometer variables
float gyro[3], gyro_average[3]; //gyroscope variables
int gyro_num_samples = 0;
float yaw, pitch, roll = 0; // Euler Angles
int curr_calibration_sensor = 0;

// DCM Variables
float mag_heading; //compass heading
float accel_vector[3] = {0, 0, 0}; //accel data vector
float gyro_vector[3] = {0, 0, 0}; //gyro data vector
float omega_vector[3] = {0, 0, 0}; //corrected gyro vector
float omega_P[3] = {0, 0, 0};
float omega_I[3] = {0, 0, 0};
float omega[3] = {0, 0, 0};
float errorRollPitch[3] = {0, 0, 0};
float errorYaw[3] = {0, 0, 0};
float DCM_Matrix[3][3], New_Matrix[3][3], Temp_Matrix[3][3];

unsigned long timestamp; //timestamp for main loop
unsigned long timestamp_old; //previous timestamp for main loop
float int_dt; // integration time

// Displacement Sensor variables
int x_int, y_int, squal; // raw and translated data
int x_real, y_real;
//int x_low,x_high,y_low,y_high;
float current_time; // timing for sensor
byte laseroff = 0; // flag for turning off laser for measurements

char OutputMode = 0; // flag for changing raw vs integrative output
// Button control variable
int button_value;

//Variables for laser controller
int laser1_power = 1500;
int laser2_power = 1500;
int num = 50;
int laser_selection = 2; //0 1 2 or 3, see DAC_mux
int LD[100] = {}; //variable to store data at first frequency
char tStr[500]; //string to output data at first frequency
int A = 6; //first pin for choosing which diode to send RF modulation to
int B = 7;
int MUX_EN = 3;
int  t1 = 0;
int  t2 = 0;

byte start_flag = 0; //added by Matt



void setup() {
  // Initialization functions
  SerialUSB.begin(output_baud_rate); //number doesn't actually matter here
  while (!SerialUSB) ; //wait for serial monitor to be opened
  delay(50);

  if (use_orientation == 1) {
    // I2C Comm, Sensors: ADXL345, HMC5883L, ITG-3200, ADNS9800
    I2C_Init();
    gyro_Init();
    magnet_Init();
    accel_Init();
  }

  if (use_displacement == 1) {
    // SPI Comm, Sensor: ADNS9800
    SPI_Init();
    lasermouse_Init(); //includes uploading firmware
    delay(20);
    reset_fusion(); //done
    delay(200);
  }

  if (use_lasers == 1) {
    analogWriteResolution(12); //change write resolution for Soroush
    //Laser controller Variables
    ADC->ADC_MR |= 0x80; // these lines set free running mode on adc 7 (pin A0)
    ADC->ADC_CR = 2;
    ADC->ADC_CHER = 0x81; // this is (1<<7) | (1<<6) for adc 7 and adc 6
    pinMode(MUX_EN, OUTPUT);
  }

  SerialUSB.println("We're ready! Send an a to start"); //let user know
}


void loop() {
  if (SerialUSB.available() >= 1) {
    if (SerialUSB.read() == 'a') {
      start_flag = 1; //send the system an a to start it
    }
    else if (SerialUSB.read() == 'b') {
      start_flag = 0; //send the system a b to stop it
    }
    else if (SerialUSB.read() == 'r') {
      reset_function();
    }
    else if (SerialUSB.read() == '!c') {
      counter=10;
    }
  }
  if (use_lasers == 1) digitalWrite(MUX_EN, HIGH ); // Lasers off

  if (start_flag == 1) { //wait to start the system until we send it an 'a'

    if ((millis() - timestamp) >= Output_Interval)
    {
      if (counter > 0)
      {

        timestamp_old = timestamp;
        timestamp = millis();
        if (timestamp > timestamp_old) {
          int_dt = (float) (timestamp - timestamp_old) / 1000.0f;
        }
        else {
          int_dt = 0;
        }

        //turn on laser diodes
        if (use_lasers == 1)
        {
          DAC_mux(laser_selection, laser1_power, laser2_power);
          t1 = micros();
          delayMicroseconds(1300);
        }

        //   update_button();

        if (use_orientation == 1)
        {
          //get tracking stuff
          read_accel();
          read_gyro();
          read_magnet();


          if (use_calibration == 1)
          {
            read_accel();
            read_gyro();
            read_magnet();
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
        }

        if (use_displacement == 1) {
          if (laseroff == 0) read_lasermouse();
        }

        if (use_lasers == 1) {
          t2 = micros();
          //output data from optical signals


          for (int i = 0; i < num; i++) //read data and save to LD and LD2 variables
          {
            //while((ADC->ADC_ISR & 0x80)==0); // wait for conversion 1channel
            while ((ADC->ADC_ISR & 0x81) != 0x81); // 2channel
            LD[i] = ADC->ADC_CDR[0]; // read data
            LD[i + num] = ADC->ADC_CDR[7];
          }

          //Convert LD and LD2 to strings (for speed - can't send array all at once)
          snprintf(tStr, 501, "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,", LD[0], LD[1], LD[2], LD[3], LD[4], LD[5], LD[6], LD[7], LD[8], LD[9], LD[10], LD[11], LD[12], LD[13], LD[14], LD[15], LD[16], LD[17], LD[18], LD[19], LD[20], LD[21], LD[22], LD[23], LD[24], LD[25], LD[26], LD[27], LD[28], LD[29], LD[30], LD[31], LD[32], LD[33], LD[34], LD[35], LD[36], LD[37], LD[38], LD[39], LD[40], LD[41], LD[42], LD[43], LD[44], LD[45], LD[46], LD[47], LD[48], LD[49], LD[50], LD[51], LD[52], LD[53], LD[54], LD[55], LD[56], LD[57], LD[58], LD[59], LD[60], LD[61], LD[62], LD[63], LD[64], LD[65], LD[66], LD[67], LD[68], LD[69], LD[70], LD[71], LD[72], LD[73], LD[74], LD[75], LD[76], LD[77], LD[78], LD[79], LD[80], LD[81], LD[82], LD[83], LD[84], LD[85], LD[86], LD[87], LD[88], LD[89], LD[90], LD[91], LD[92], LD[93], LD[94], LD[95], LD[96], LD[97], LD[98], LD[99]);
        }
        //output results
        if (output_all == 1)
        {
          output_print_all(); // print all variables to output SerialUSB
        }
        else {
          output_print();  // print output data to SerialUSB
        }
        if (use_counter==1) counter--;
      }
      else {}
    }
    else {}
  }//if not started, do nothing
}
//end of loop

void output_print() {
  SerialUSB.print(millis());
  SerialUSB.print(",");
  if (use_displacement == 1)
  {
    SerialUSB.print(x_real);
    SerialUSB.print(",");
    SerialUSB.print(y_real);
    SerialUSB.print(",");
    SerialUSB.print(squal);
    SerialUSB.print(",");
  }
  if (use_orientation == 1)
  {
    SerialUSB.print(to_deg(yaw));
    SerialUSB.print(",");
    SerialUSB.print(to_deg(pitch));
    SerialUSB.print(",");
    SerialUSB.print(to_deg(roll));
    SerialUSB.print(",");
    
  }
  //  SerialUSB.print(button_value);
  //  SerialUSB.print(",");
  
  if (use_lasers == 1)
  {
    SerialUSB.print(t2 - t1);
    SerialUSB.print(",");
    SerialUSB.print(tStr);
  }
  SerialUSB.println(" ");
}

void output_print_all() {
  SerialUSB.print(millis());
  SerialUSB.print(",");
  SerialUSB.print(x_real);
  SerialUSB.print(",");
  SerialUSB.print(y_real);
  SerialUSB.print(",");
  SerialUSB.print(to_deg(yaw));
  SerialUSB.print(",");
  SerialUSB.print(to_deg(pitch));
  SerialUSB.print(",");
  SerialUSB.print(to_deg(roll));
  SerialUSB.print(",");
//  SerialUSB.print(button_value);
//  SerialUSB.print(",");
  SerialUSB.print(squal);
  SerialUSB.print(",");
  SerialUSB.print(t2 - t1);
  SerialUSB.print(",");
  SerialUSB.print(tStr);
  SerialUSB.println(" ");
}

void update_button() {
  button_value = analogRead(0);
}

void reset_function() {
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);
}


//laser controller function
void DAC_mux(int LD, int pow1, int pow2)
{
  pinMode(A, OUTPUT); //pin for the first laser diode, pin number set above
  pinMode(B, OUTPUT); //pin for second diode, set above
  digitalWrite(MUX_EN, LOW);

  switch (LD) //0, 1, 2, or 3 sent to the function
  {
    case 0: //turn on neither diode
      digitalWrite(A, LOW);
      digitalWrite(B, LOW);
      break;
    case 1: //turn on first diode only
      digitalWrite(A, HIGH);
      digitalWrite(B, LOW);
      break;
    case 2: //turn on second diode only
      digitalWrite(A, LOW);
      digitalWrite(B, HIGH);
      break;
    case 3: //turn on both diodes
      digitalWrite(A, HIGH);
      digitalWrite(B, HIGH);
      break;
    default: //do nothing if something else sent
      break;
  }//end of MUX selection

  analogWrite(DAC0, pow1); //set modulation amplitude for laser diode 1?
  analogWrite(DAC1, pow2); //set modulation amplitude for laser diode 2?

} //end of DAC_mux function
