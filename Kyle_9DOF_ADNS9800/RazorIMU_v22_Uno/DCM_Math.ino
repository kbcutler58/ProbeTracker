void reset_fusion() {
  float temp1[3];
  float temp2[3];
  float xAxis[] = {1.0f, 0.0f, 0.0f};

  read_accel();
  read_gyro();
  read_magnet();
  timestamp = millis();
  
  pitch = -atan2(accel[0], sqrt(accel[1] * accel[1] + accel[2] * accel[2]));
	
  Vector_Cross_Product(temp1, accel, xAxis);
  Vector_Cross_Product(temp2, xAxis, temp1);
  roll = atan2(temp2[1], temp2[2]);
  
  // GET YAW
  compass_heading();
  yaw = mag_heading;
  
  // Init rotation matrix
  rotation_matrix_Init(DCM_Matrix, yaw, pitch, roll);
}

void compensate_errors() {

  // Scale accelerometer data
  accel[0] = (accel[0] - accel_x_off) * accel_x_scale;
  accel[1] = (accel[1] - accel_y_off) * accel_y_scale;
  accel[2] = (accel[2] - accel_z_off) * accel_z_scale; 
  // Scale gyroscope data
  gyro[0] -= -gyro_offset_x;
  gyro[1] -= -gyro_offset_y;
  gyro[2] -= -gyro_offset_z;
    
  // Scale compass data
  magnet[0] = (magnet[0] - magnet_x_off) * magnet_x_scale;
  magnet[1] = (magnet[1] - magnet_y_off) * magnet_y_scale;
  magnet[2] = (magnet[2] - magnet_z_off) * magnet_z_scale;
  
}

// Init rotation matrix using euler angles
void rotation_matrix_Init(float m[3][3], float yaw, float pitch, float roll)
{
  float c1 = cos(roll);
  float s1 = sin(roll);
  float c2 = cos(pitch);
  float s2 = sin(pitch);
  float c3 = cos(yaw);
  float s3 = sin(yaw);

  // Euler angles, right-handed, intrinsic, XYZ convention
  // (which means: rotate around body axes Z, Y', X'') 
  m[0][0] = c2 * c3;
  m[0][1] = c3 * s1 * s2 - c1 * s3;
  m[0][2] = s1 * s3 + c1 * c3 * s2;

  m[1][0] = c2 * s3;
  m[1][1] = c1 * c3 + s1 * s2 * s3;
  m[1][2] = c1 * s2 * s3 - c3 * s1;

  m[2][0] = -s2;
  m[2][1] = c2 * s1;
  m[2][2] = c1 * c2;
}

void normalize_values(void)
{
  float error=0;
  float temporary[3][3];
  float renorm=0;
  
  error= -Vector_Dot_Product(&DCM_Matrix[0][0],&DCM_Matrix[1][0])*.5; //eq.19

  Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
  Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19
  
  Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
  Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19
  
  Vector_Cross_Product(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq.20
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[0][0],&temporary[0][0])); //eq.21
  Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[1][0],&temporary[1][0])); //eq.21
  Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[2][0],&temporary[2][0])); //eq.21
  Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
}

void drift_correction(void)
{
  float mag_heading_x;
  float mag_heading_y;
  float errorCourse;
  //Compensation the Roll, Pitch and Yaw drift. 
  static float scaled_omega_P[3];
  static float scaled_omega_I[3];
  float accel_magnitude;
  float accel_weight;
  
  
  //*****Roll and Pitch***************

  // Calculate the magnitude of the accelerometer vector
  accel_magnitude = sqrt(accel_vector[0]*accel_vector[0] + accel_vector[1]*accel_vector[1] + accel_vector[2]*accel_vector[2]);
  accel_magnitude = accel_magnitude / gravity; // Scale to gravity.
  // Dynamic weighting of accelerometer info (reliability filter)
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  accel_weight = constrain(1 - 2*abs(1 - accel_magnitude),0,1); 

  Vector_Cross_Product(&errorRollPitch[0],&accel_vector[0],&DCM_Matrix[2][0]); //adjust the ground of reference
  Vector_Scale(&omega_P[0],&errorRollPitch[0],kp_Rollpitch*accel_weight);
  
  Vector_Scale(&scaled_omega_I[0],&errorRollPitch[0],ki_Rollpitch*accel_weight);
  Vector_Add(omega_I,omega_I,scaled_omega_I);     
  
  //*****YAW***************
  // We make the gyro YAW drift correction based on compass magnetic heading
 
  mag_heading_x = cos(mag_heading);
  mag_heading_y = sin(mag_heading);
  errorCourse=(DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);  //Calculating YAW error
  Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation, depeding the position.
  
  Vector_Scale(&scaled_omega_P[0],&errorYaw[0],kp_Yaw);//.01proportional of YAW.
  Vector_Add(omega_P,omega_P,scaled_omega_P);//Adding  Proportional.
  
  Vector_Scale(&scaled_omega_I[0],&errorYaw[0],ki_Yaw);//.00001Integrator
  Vector_Add(omega_I,omega_I,scaled_omega_I);//adding integrator to the Omega_I
}

void matrix_update(void)
{
  gyro_vector[0]=gyro_scaled_rad(gyro[0]); //gyro x roll
  gyro_vector[1]=gyro_scaled_rad(gyro[1]); //gyro y pitch
  gyro_vector[2]=gyro_scaled_rad(gyro[2]); //gyro z yaw
  
  accel_vector[0]=accel[0];
  accel_vector[1]=accel[1];
  accel_vector[2]=accel[2];
    
  Vector_Add(&omega[0], &gyro_vector[0], &omega_I[0]);  //adding proportional term
  Vector_Add(&omega_vector[0], &omega[0], &omega_P[0]); //adding Integrator term
  
  New_Matrix[0][0]=0;
  New_Matrix[0][1]=-int_dt*omega_vector[2];//-z
  New_Matrix[0][2]=int_dt*omega_vector[1];//y
  New_Matrix[1][0]=int_dt*omega_vector[2];//z
  New_Matrix[1][1]=0;
  New_Matrix[1][2]=-int_dt*omega_vector[0];//-x
  New_Matrix[2][0]=-int_dt*omega_vector[1];//-y
  New_Matrix[2][1]=int_dt*omega_vector[0];//x
  New_Matrix[2][2]=0;

  Matrix_Multiply(DCM_Matrix,New_Matrix,Temp_Matrix); //a*b=c

  for(int x=0; x<3; x++) //Matrix Addition (update)
  {
    for(int y=0; y<3; y++)
    {
      DCM_Matrix[x][y]+=Temp_Matrix[x][y];
    } 
  }
}

void convert_angles()
{
  pitch = -asin(DCM_Matrix[2][0]);
  roll = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
  yaw = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
}

float Vector_Dot_Product(const float v1[3], const float v2[3])
{
  float result = 0;
  
  for(int c = 0; c < 3; c++)
  {
    result += v1[c] * v2[c];
  }
  
  return result; 
}

void Vector_Cross_Product(float out[3], const float v1[3], const float v2[3])
{
  out[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
  out[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
  out[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
}

void Vector_Scale(float out[3], const float v[3], float scale)
{
  for(int c = 0; c < 3; c++)
  {
    out[c] = v[c] * scale; 
  }
}

void Vector_Add(float out[3], const float v1[3], const float v2[3])
{
  for(int c = 0; c < 3; c++)
  {
    out[c] = v1[c] + v2[c];
  }
}

void Matrix_Multiply(const float a[3][3], const float b[3][3], float out[3][3])
{
  for(int x = 0; x < 3; x++)  // rows
  {
    for(int y = 0; y < 3; y++)  // columns
    {
      out[x][y] = a[x][0] * b[0][y] + a[x][1] * b[1][y] + a[x][2] * b[2][y];
    }
  }
}

void Matrix_Vector_Multiply(const float a[3][3], const float b[3], float out[3])
{
  for(int x = 0; x < 3; x++)
  {
    out[x] = a[x][0] * b[0] + a[x][1] * b[1] + a[x][2] * b[2];
  }
}



