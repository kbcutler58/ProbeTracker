// Take in raw magnetometer input
// Create text file of points

import processing.serial.*;
PrintWriter logger;

Serial serial;
final String serialPort = "COM11";

String inputString;
String magx,magy,magz;


void setup()
{
  size(300,300);
  println("available serial ports:");
  println(Serial.list());
  serial = new Serial(this, serialPort, 115200);
  logger = createWriter("Magnet_Cal_Data3.txt");
  delay(100);
}

void draw()
{
  if (serial.available() >= 17) {
    inputString = serial.readStringUntil('\n');
    if (inputString != null && inputString.length() > 0) {
      String [] inputStringArr = split(inputString, ",");
      if (inputStringArr.length >= 3) {
        magx = inputStringArr[0];
        magy = inputStringArr[1];
        magz = inputStringArr[2];
        logger.print(magx);
        logger.print(",");
        logger.print(magy);
        logger.print(",");
        logger.print(magz);
        logger.println(" ");
      }
    }
  }
  delay(10);
}  

void keyPressed() {
  if (key =='c') {
    delay(100);
    logger.flush();
    logger.close();
    exit();
  }
}
  
  
  
