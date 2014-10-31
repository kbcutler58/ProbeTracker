// take in (x,y,theta) (in degrees)
float x_motion,y_motion,theta;
float x_motion_old = 0;
float y_motion_old = 0;
float x_prime = 0;
float y_prime = 0;
float x_float, y_float, x_diff, y_diff;
float x_global = 0;
float y_global = 0;

void Tracker2D_mousedata_Transform()
{
  x_float = float(x);
  y_float = float(y);
  if (x_float > 128) x_float -= 256;
  if (y_float > 128) y_float -= 256;
  x_motion += x_float;
  y_motion += y_float;
  //if x or y > 128 x or y =  x or y - 255
}

void Tracker2D_xy_Transform()
{
  theta = yaw;
  x_diff = x_motion - x_motion_old;
  y_diff = y_motion - y_motion_old;
  x_prime = x_diff*cos(theta)-y_diff*sin(theta);
  y_prime = x_diff*sin(theta)+y_diff*cos(theta);
  x_motion_old = x_motion;
  y_motion_old = y_motion;
  x_global += x_prime;
  y_global += y_prime;
}
  // output (x', y')

void Tracker2D_output()
{
//Print 2D position  to serial

// Added lines for debugging
//  Serial.print(x_float); Serial.print(","); Serial.print(y_float); Serial.print(","); 
//  Serial.print(x_motion); Serial.print(","); Serial.print(x_motion); Serial.print(",");
//  Serial.print(x_prime); Serial.print(",");  Serial.print(y_prime); Serial.print(",");
  Serial.print(x_global); Serial.print(",");  Serial.print(y_global); Serial.print(",");
  
//  Serial.print(theta); Serial.print(",");
  Serial.print(squal); Serial.print(",");
}

