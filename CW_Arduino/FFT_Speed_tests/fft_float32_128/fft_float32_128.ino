
#define ARM_MATH_CM3
#include <arm_math.h>

#define TEST_LENGTH_SAMPLES 128 



static float32_t testOutput[TEST_LENGTH_SAMPLES/2]; 

uint32_t M = 0;

/* ------------------------------------------------------------------ 
* Global variables for FFT Bin Example 
* ------------------------------------------------------------------- */ 
uint32_t fftSize = 64; 
uint32_t ifftFlag = 0; 
uint32_t doBitReverse = 1; 

/* Reference index at which max energy of bin ocuurs */ 
uint32_t refIndex = 213, testIndex = 0; 

/* ----------------------------------------------------------------------
Test Input signal contains 10KHz signal + Uniformly distributed white noise
** ------------------------------------------------------------------- */

float32_t testInput_f32_10khz_256[128] {
-0.865129623056441, 	0.000000000000000, 	-2.655020678073846, 	0.000000000000000, 	0.600664612949661, 	0.000000000000000, 	0.080378093886515, 	0.000000000000000, 	
-2.899160484012034, 	0.000000000000000, 	2.563004262857762, 	0.000000000000000, 	3.078328403304206, 	0.000000000000000, 	0.105906778385130, 	0.000000000000000, 	
0.048366940168201, 	0.000000000000000, 	-0.145696461188734, 	0.000000000000000, 	-0.023417155362879, 	0.000000000000000, 	2.127729174988954, 	0.000000000000000, 	
-1.176633086028377, 	0.000000000000000, 	3.690223557991855, 	0.000000000000000, 	-0.622791766173194, 	0.000000000000000, 	0.722837373872203, 	0.000000000000000, 	
2.739754205367484, 	0.000000000000000, 	-0.062610410524552, 	0.000000000000000, 	-0.891296810967338, 	0.000000000000000, 	-1.845872258871811, 	0.000000000000000, 	
1.195039415434387, 	0.000000000000000, 	-2.177388969045026, 	0.000000000000000, 	1.078649103637905, 	0.000000000000000, 	2.570976050490193, 	0.000000000000000, 	
-1.383551403404574, 	0.000000000000000, 	2.392141424058873, 	0.000000000000000, 	2.858002843205065, 	0.000000000000000, 	-3.682433899725536, 	0.000000000000000, 	
-3.488146646451150, 	0.000000000000000, 	1.323468578888120, 	0.000000000000000, 	-0.099771155430726, 	0.000000000000000, 	1.561168082500454, 	0.000000000000000, 	
1.025026795103179, 	0.000000000000000, 	0.928841900171200, 	0.000000000000000, 	2.930499509864950, 	0.000000000000000, 	2.013349089766430, 	0.000000000000000, 	
2.381676148486737, 	0.000000000000000, 	-3.081062307950236, 	0.000000000000000, 	-0.389579115537544, 	0.000000000000000, 	0.181540149166620, 	0.000000000000000, 	
-2.601953341353208, 	0.000000000000000, 	0.333435137783218, 	0.000000000000000, 	-2.812945856162965, 	0.000000000000000, 	2.649109640172910, 	0.000000000000000, 	
-1.003963025744654, 	0.000000000000000, 	1.552460768755035, 	0.000000000000000, 	0.088641345335247, 	0.000000000000000, 	-2.519951327113426, 	0.000000000000000, 	
-4.341348988610527, 	0.000000000000000, 	0.557772429359965, 	0.000000000000000, 	-1.671267412948494, 	0.000000000000000, 	0.733951350960387, 	0.000000000000000, 	
0.409263788034864, 	0.000000000000000, 	3.566033071952806, 	0.000000000000000, 	1.882565173848352, 	0.000000000000000, 	-1.106017073793287, 	0.000000000000000, 	
0.154456720778718, 	0.000000000000000, 	-2.513205795512153, 	0.000000000000000, 	0.310978660939421, 	0.000000000000000, 	0.579706500111723, 	0.000000000000000, 	
0.000086383683251, 	0.000000000000000, 	-1.311866980897721, 	0.000000000000000, 	1.840007477574986, 	0.000000000000000, 	-3.253005768451345, 	0.000000000000000, 	
};

int t1,t2,t3,t4,t5;
void setup() {
 SerialUSB.begin(19200);
 while (!SerialUSB);
 SerialUSB.println(" start program ");
 delay(50);
}

void loop() {
      
    
      /** \example arm_fft_bin_example_f32.c 
 */  

	arm_status status; 
	arm_cfft_radix4_instance_f32 S; 
	float32_t maxValue; 
	 
	status = ARM_MATH_SUCCESS; 
	/* Initialize the CFFT/CIFFT module */
        t1 = micros();
	status = arm_cfft_radix4_init_f32(&S, fftSize, ifftFlag, doBitReverse); 
//	SerialUSB.println("step 1"); 
        t2 = micros();
	/* Process the data through the CFFT/CIFFT module */ 
	arm_cfft_radix4_f32(&S, testInput_f32_10khz_256); 
//       	SerialUSB.println("Status1"); 
//	SerialUSB.println("step 2");  
	t3 = micros();
	/* Process the data through the Complex Magnitude Module for  
	calculating the magnitude at each bin */ 
	arm_cmplx_mag_f32(testInput_f32_10khz_256, testOutput, fftSize);  
        t4 = micros();
//	SerialUSB.println("step 3");  
	/* Calculates maxValue and returns corresponding BIN value */ 
	arm_max_f32(testOutput, fftSize, &maxValue, &testIndex); 
        t5 = micros();
        SerialUSB.print(t1);
        SerialUSB.print(",");
        SerialUSB.print(t2-t1);
        SerialUSB.print(",");
        SerialUSB.print(t3-t2);
        SerialUSB.print(",");
        SerialUSB.print(t4-t3);
        SerialUSB.print(",");
        SerialUSB.println(t5-t4);
//	SerialUSB.println("step 4");  
       SerialUSB.println(testIndex,DEC);
       SerialUSB.println(refIndex,DEC);       

while(1);

                                /* main function does not return */
   
}
