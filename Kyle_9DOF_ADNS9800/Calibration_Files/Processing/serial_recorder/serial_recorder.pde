// Take in raw magnetometer input
// Create text file of points

import processing.serial.*;
PrintWriter logger;

Serial serial;
final String serialPort = "COM12";

String inputString;
String magx,magy,magz;
String time,x,y,yaw,pitch,roll,button;

void setup()
{
  size(300,300);
  println("available serial ports:");
  println(Serial.list());
  serial = new Serial(this, serialPort, 115200);
  logger = createWriter("OrientationNoise1.txt");
  delay(2000);
}

void draw()
{
  if (serial.available() >= 61) {
    inputString = serial.readStringUntil('\n');
    if (inputString != null && inputString.length() > 0) {
      String [] inputStringArr = split(inputString, ",");
      if (inputStringArr.length >= 6) {
        time = inputStringArr[0];
        x = inputStringArr[1];
        y = inputStringArr[2];
        yaw = inputStringArr[3];
        pitch = inputStringArr[4];
        roll = inputStringArr[5];
        button = inputStringArr[6];
        
//        magx = inputStringArr[0];
//        magy = inputStringArr[1];
//        magz = inputStringArr[2];
//        logger.print(magx);
//        logger.print(",");
//        logger.print(magy);
//        logger.print(",");
//        logger.print(magz);
//        logger.println(" ");


        logger.print(time);
        logger.print(",");
        logger.print(x);
        logger.print(",");
        logger.print(y);
        logger.print(",");
        logger.print(yaw);
        logger.print(",");
        logger.print(pitch);
        logger.print(",");
        logger.print(roll);
        logger.print(",");
        logger.print(button);
        logger.println(" ");
      }
    }
  }
  delay(10);
}  

void keyPressed() {
  if (key =='c') {
    delay(500);
    logger.flush();
    logger.close();
    exit();
  }//000103691,000000000,000000000,-108.152,0003.514,0007.961
}
  
  
  
