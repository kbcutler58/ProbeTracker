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
#define Output_Interval 18
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

//#define gyro_offset_x ((float) -10.45)
//#define gyro_offset_y ((float) 59.60)
//#define gyro_offset_z ((float) 4.99)

//
//// Calibration values
//#define accel_x_min ((float) -283)
//#define accel_y_min ((float) -256)
//#define accel_z_min ((float) -264)
//#define accel_x_max ((float) 249)
//#define accel_y_max ((float) 268)
//#define accel_z_max ((float) 247)
//
//#define magnet_x_min ((float) -1393)
//#define magnet_y_min ((float) -1308)
//#define magnet_z_min ((float) -1160)
//#define magnet_x_max ((float) 1615)
//#define magnet_y_max ((float) 1643)
//#define magnet_z_max ((float) 4096)
//
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

// Program Options (Change to 0 to turn off)
byte use_orientation = 1; // Will turn on and off i2c and orientation measurements
byte use_displacement = 1; // Will turn on and off SPI and displacement measurements
byte use_lasers = 1; // Will turn on CW laser measurements
byte output_lasers = 1;
//byte output_all = 1; // Will output all data even if off
byte use_calibration = 0;
byte use_counter = 0;
int counter = 0;
byte serial_control = 0; // Set to 1 if using CVI to call each measurement
byte start_flag = 1; // controls auto starting 1:autostart 0:send 'a' to start

// Orientation Sensor variables
float accel[3], accel_min[3], accel_max[3]; //accelerometer variables
float magnet[3], magnet_min[3], magnet_max[3], magnet_temp[3]; //magnetometer variables
float gyro[3], gyro_average[3]; //gyroscope variables
int gyro_num_samples = 0;
float yaw, pitch, roll = 0; // Euler Angles
int curr_calibration_sensor = 0;

// Extended Magnet Calibration
//const float magnet_ellipsoid_center[3] = {-14.6, 104.0, 22.2};
//const float magnet_ellipsoid_transform[3][3] = {{3.3335E-6, 3.5204E-6 , 4.1310E-6 }, {-1.1816E-8, -1.2134E-8, 3.6146E-8}, {5.0396E-5, -3.6931E-4, -9.5678E-5}};
char button_string[10];

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

// Button control variable
int button_value;

//Variables for laser controller
int laser1_power = 2350;
int laser2_power = 2350;
int low_power = 400;
int num = 100;

//int laser_selection = 2; //0 1 2 or 3, see DAC_mux

int LD1[200] = {}; //variable to store data at first frequency
int LD2[200] = {};
char tStr[1010]; //string to output data at first frequency
char tStr2[1010];
int A = 6; //first pin for choosing which diode to send RF modulation to
int B = 7;
int MUX_EN = 3;
int  t1 = 0;
int  t2 = 0;

char outputString[100];


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
    delay(20);
    reset_fusion(); //done
    delay(200);
  }

  if (use_displacement == 1) {
    // SPI Comm, Sensor: ADNS9800
    SPI_Init();
    lasermouse_Init(); //includes uploading firmware
    delay(50);
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
      digitalWrite(MUX_EN, HIGH);
    }
    else if (SerialUSB.read() == 'r') {
      reset_function();
    }
    else if (SerialUSB.read() == '!c') {
      counter = 10;
    }
  }

  // wait to start the system until we send it an 'a'
  // controlled with start_flag variable
  if (start_flag == 1) {

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
      if (counter == 0) {
        DAC_mux(2, laser1_power, laser2_power);
        delay(20);
      }
      else { }

      //        if (use_lasers == 1)
      //        {
      //          DAC_mux(laser_selection, laser1_power, laser2_power);
      //          t1 = micros();
      //          if ((use_orientation == 0) && (use_displacement == 0)) delayMicroseconds(1300);
      //          //         if ((use_orientation = 0) && (use_displacement == 0))
      //        }

      if (use_orientation == 1)
      {
        //get tracking stuff
        read_accel();
        read_gyro();
        read_magnet();
        update_button();


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
        if (isnan(to_deg(yaw))==1) reset_fusion();
      }

      if (use_displacement == 1) {
        if (laseroff == 0) read_lasermouse();
      }

      if (use_lasers == 1) {
        t2 = micros();
        //output data from optical signals

        for (int i = 0; i < num; i++)
        {
          while ((ADC->ADC_ISR & 0x80) == 0); // wait for conversion 1channel
          //       while((ADC->ADC_ISR & 0x81)!=0x81); // 2channel
          LD1[i] = (ADC->ADC_CDR[7]);   // read data
          delayMicroseconds(1);
          //      LD1[i+num]=ADC->ADC_CDR[0];
        }
        delayMicroseconds(1);
        //
        for (int i = 0; i < num; i++)
        {
          while ((ADC->ADC_ISR & 0x01) == 0); // wait for conversion 1channel
          //       while((ADC->ADC_ISR & 0x81)!=0x81); // 2channel
          LD1[i + num] = (ADC->ADC_CDR[0]); // read data
          delayMicroseconds(1);
          //      LD1[i+num]=ADC->ADC_CDR[0];
        }

        //Convert LD and LD2 to strings (for speed - can't send array all at once)
        //    snprintf(tStr,501,"%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,",LD[0],LD[1],LD[2],LD[3],LD[4],LD[5],LD[6],LD[7],LD[8],LD[9], LD[10], LD[11],LD[12],LD[13],LD[14],LD[15],LD[16],LD[17],LD[18],LD[19], LD[20], LD[21],LD[22],LD[23],LD[24],LD[25],LD[26],LD[27],LD[28],LD[29], LD[30], LD[31],LD[32],LD[33],LD[34],LD[35],LD[36],LD[37],LD[38],LD[39], LD[40], LD[41],LD[42],LD[43],LD[44],LD[45],LD[46],LD[47],LD[48],LD[49], LD[50], LD[51],LD[52],LD[53],LD[54],LD[55],LD[56],LD[57],LD[58],LD[59], LD[60], LD[61],LD[62],LD[63],LD[64],LD[65],LD[66],LD[67],LD[68],LD[69], LD[70], LD[71],LD[72],LD[73],LD[74],LD[75],LD[76],LD[77],LD[78],LD[79], LD[80], LD[81],LD[82],LD[83],LD[84],LD[85],LD[86],LD[87],LD[88],LD[89], LD[90], LD[91],LD[92],LD[93],LD[94],LD[95],LD[96],LD[97],LD[98],LD[99]);
        snprintf(tStr, 1001, "%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,", LD1[0], LD1[1], LD1[2], LD1[3], LD1[4], LD1[5], LD1[6], LD1[7], LD1[8], LD1[9], LD1[10], LD1[11], LD1[12], LD1[13], LD1[14], LD1[15], LD1[16], LD1[17], LD1[18], LD1[19], LD1[20], LD1[21], LD1[22], LD1[23], LD1[24], LD1[25], LD1[26], LD1[27], LD1[28], LD1[29], LD1[30], LD1[31], LD1[32], LD1[33], LD1[34], LD1[35], LD1[36], LD1[37], LD1[38], LD1[39], LD1[40], LD1[41], LD1[42], LD1[43], LD1[44], LD1[45], LD1[46], LD1[47], LD1[48], LD1[49], LD1[50], LD1[51], LD1[52], LD1[53], LD1[54], LD1[55], LD1[56], LD1[57], LD1[58], LD1[59], LD1[60], LD1[61], LD1[62], LD1[63], LD1[64], LD1[65], LD1[66], LD1[67], LD1[68], LD1[69], LD1[70], LD1[71], LD1[72], LD1[73], LD1[74], LD1[75], LD1[76], LD1[77], LD1[78], LD1[79], LD1[80], LD1[81], LD1[82], LD1[83], LD1[84], LD1[85], LD1[86], LD1[87], LD1[88], LD1[89], LD1[90], LD1[91], LD1[92], LD1[93], LD1[94], LD1[95], LD1[96], LD1[97], LD1[98], LD1[99], LD1[100], LD1[101], LD1[102], LD1[103], LD1[104], LD1[105], LD1[106], LD1[107], LD1[108], LD1[109], LD1[110], LD1[111], LD1[112], LD1[113], LD1[114], LD1[115], LD1[116], LD1[117], LD1[118], LD1[119], LD1[120], LD1[121], LD1[122], LD1[123], LD1[124], LD1[125], LD1[126], LD1[127], LD1[128], LD1[129], LD1[130], LD1[131], LD1[132], LD1[133], LD1[134], LD1[135], LD1[136], LD1[137], LD1[138], LD1[139], LD1[140], LD1[141], LD1[142], LD1[143], LD1[144], LD1[145], LD1[146], LD1[147], LD1[148], LD1[149], LD1[150], LD1[151], LD1[152], LD1[153], LD1[154], LD1[155], LD1[156], LD1[157], LD1[158], LD1[159], LD1[160], LD1[161], LD1[162], LD1[163], LD1[164], LD1[165], LD1[166], LD1[167], LD1[168], LD1[169], LD1[170], LD1[171], LD1[172], LD1[173], LD1[174], LD1[175], LD1[176], LD1[177], LD1[178], LD1[179], LD1[180], LD1[181], LD1[182], LD1[183], LD1[184], LD1[185], LD1[186], LD1[187], LD1[188], LD1[189], LD1[190], LD1[191], LD1[192], LD1[193], LD1[194], LD1[195], LD1[196], LD1[197], LD1[198], LD1[199]);

        DAC_mux(2, low_power, low_power);
        DAC_mux(1, laser1_power, laser2_power);

        //output all except last two strings
        output_print();

        for (int i = 0; i < num; i++)
        {
          while ((ADC->ADC_ISR & 0x80) == 0); // wait for conversion 1channel
          //       while((ADC->ADC_ISR & 0x81)!=0x81); // 2channel
          LD2[i] = (ADC->ADC_CDR[7]);   // read data
          delayMicroseconds(1);
          //      LD1[i+num]=ADC->ADC_CDR[0];
        }
        delayMicroseconds(1);
        //
        for (int i = 0; i < num; i++)
        {
          while ((ADC->ADC_ISR & 0x01) == 0); // wait for conversion 1channel
          //       while((ADC->ADC_ISR & 0x81)!=0x81); // 2channel
          LD2[i + num] = (ADC->ADC_CDR[0]); // read data
          delayMicroseconds(1);
          //      LD1[i+num]=ADC->ADC_CDR[0];
        }

        snprintf(tStr2, 1000, "%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d", LD2[0], LD2[1], LD2[2], LD2[3], LD2[4], LD2[5], LD2[6], LD2[7], LD2[8], LD2[9], LD2[10], LD2[11], LD2[12], LD2[13], LD2[14], LD2[15], LD2[16], LD2[17], LD2[18], LD2[19], LD2[20], LD2[21], LD2[22], LD2[23], LD2[24], LD2[25], LD2[26], LD2[27], LD2[28], LD2[29], LD2[30], LD2[31], LD2[32], LD2[33], LD2[34], LD2[35], LD2[36], LD2[37], LD2[38], LD2[39], LD2[40], LD2[41], LD2[42], LD2[43], LD2[44], LD2[45], LD2[46], LD2[47], LD2[48], LD2[49], LD2[50], LD2[51], LD2[52], LD2[53], LD2[54], LD2[55], LD2[56], LD2[57], LD2[58], LD2[59], LD2[60], LD2[61], LD2[62], LD2[63], LD2[64], LD2[65], LD2[66], LD2[67], LD2[68], LD2[69], LD2[70], LD2[71], LD2[72], LD2[73], LD2[74], LD2[75], LD2[76], LD2[77], LD2[78], LD2[79], LD2[80], LD2[81], LD2[82], LD2[83], LD2[84], LD2[85], LD2[86], LD2[87], LD2[88], LD2[89], LD2[90], LD2[91], LD2[92], LD2[93], LD2[94], LD2[95], LD2[96], LD2[97], LD2[98], LD2[99], LD2[100], LD2[101], LD2[102], LD2[103], LD2[104], LD2[105], LD2[106], LD2[107], LD2[108], LD2[109], LD2[110], LD2[111], LD2[112], LD2[113], LD2[114], LD2[115], LD2[116], LD2[117], LD2[118], LD2[119], LD2[120], LD2[121], LD2[122], LD2[123], LD2[124], LD2[125], LD2[126], LD2[127], LD2[128], LD2[129], LD2[130], LD2[131], LD2[132], LD2[133], LD2[134], LD2[135], LD2[136], LD2[137], LD2[138], LD2[139], LD2[140], LD2[141], LD2[142], LD2[143], LD2[144], LD2[145], LD2[146], LD2[147], LD2[148], LD2[149], LD2[150], LD2[151], LD2[152], LD2[153], LD2[154], LD2[155], LD2[156], LD2[157], LD2[158], LD2[159], LD2[160], LD2[161], LD2[162], LD2[163], LD2[164], LD2[165], LD2[166], LD2[167], LD2[168], LD2[169], LD2[170], LD2[171], LD2[172], LD2[173], LD2[174], LD2[175], LD2[176], LD2[177], LD2[178], LD2[179], LD2[180], LD2[181], LD2[182], LD2[183], LD2[184], LD2[185], LD2[186], LD2[187], LD2[188], LD2[189], LD2[190], LD2[191], LD2[192], LD2[193], LD2[194], LD2[195], LD2[196], LD2[197], LD2[198], LD2[199]);
        DAC_mux(1, low_power, low_power);
        DAC_mux(2, laser1_power, laser2_power);
        output_print2();
        counter = 1;

        if (serial_control == 1) start_flag = 0;
      }
      else {  }
    }
    else { }
  }

  //if not started, do nothing
}
//end of loop

void output_print() {
  snprintf(outputString, 100, "%09d,%09d,%09d,%08.3f,%08.3f,%08.3f", millis(), x_real, y_real, to_deg(yaw), to_deg(pitch), to_deg(roll));
  SerialUSB.print(outputString);
//  SerialUSB.print(",");
//  SerialUSB.print(button_value);
  if (output_lasers==1) {SerialUSB.print(tStr); SerialUSB.print(",");}
}

void output_print2() {
 if (output_lasers==1) SerialUSB.print(tStr2);
  SerialUSB.println();
}

void update_button() {
  button_value = analogRead(0);
  snprintf(button_string, 4, "%04d", button_value);
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
