#include <Wire.h> //Orientation Comm Protocol
#include <avr/pgmspace.h>

//definitions
#define Output_Interval 50
#define output_baud_rate 115200

#define to_deg(x) (x * 57.2957795131)
#define to_reg(x) (x * 0.01745329252)

// Program Options (Change to 0 to turn off)
int sensor_select = 0; // 0=compass 1=accel 2=gryo

// Orientation Sensor variables
float accel[3], accel_min[3], accel_max[3]; //accelerometer variables
float magnet[3], magnet_min[3], magnet_max[3], magnet_temp[3]; //magnetometer variables
float gyro[3], gyro_average[3]; //gyroscope variables
int gyro_num_samples = 0;

unsigned long timestamp; //timestamp for main loop
unsigned long timestamp_old; //previous timestamp for main loop

float current_time; // timing for sensor
byte auto_start = 1; //added by Matt

void setup() {
  // Initialization functions
  SerialUSB.begin(output_baud_rate); //number doesn't actually matter here
  while (!SerialUSB) ; //wait for serial monitor to be opened
  delay(50);
  
    // I2C Comm, Sensors: ADXL345, HMC5883L, ITG-3200, ADNS9800
    I2C_Init();
    gyro_Init();
    magnet_Init();
    accel_Init();
    delay(200);

  SerialUSB.println("Start"); //let user know
}


void loop() {

  if (auto_start == 1)
  {
    switch(sensor_select)
    {
      case 0: //magnetometer calibration
        read_magnet();
        for (int i = 0; i<3; i++){
          SerialUSB.print(magnet[i]);
          if (i < 2) SerialUSB.print(",");
          else SerialUSB.println();  
        }
        break;
      
      case 1:
        read_accel();
        
        for (int i = 0; i<3; i++) {
          SerialUSB.print(accel[i]);
          if (i < 2) SerialUSB.print(",");
          else SerialUSB.println();  
        }
        break;
        
      case 2:
        read_gyro();
        for (int i = 0; i<3; i++) {
          SerialUSB.print(gyro[i]);
          if (i < 2) SerialUSB.print(",");
          else SerialUSB.println();  
        }
        break;
      default:
        SerialUSB.print("nothing");
        break;
    }
    
    delay(Output_Interval);
  }
}



