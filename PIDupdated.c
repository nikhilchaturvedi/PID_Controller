
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#define ARRAY_LEN 4096    //define array length 
#define KERNEL_LEN 7      //define kernel length 
#define ERROR_LEN 4096    //define error length 
#define pi 3.1415926      //   

#define alpha_p 1   //coefficient for proporation controller for later intelligent control   
#define alpha_i 1   //coefficient for integral controller for later intelligent control   
#define alpha_d 1   //coefficient for derivative controller for later intelligent control   

#define f_pwm   1500  //PWM frequency for stepper motor steering which gives 
//66.67 uSec sampling time interval  

#define microStepFull 1.8    //full  
#define microStepHalf 0.9    //half  
#define microStepQ    0.45   //quarter  
#define microStep8    0.225  //one eighth  

#define speedVe    5000  //5 Km per hour, or 1.389 meter per second   

#define PWM_IOCTL_SET_FREQ		1
#define PWM_IOCTL_STOP			0

#define M 5
#define N 32

void  getDerive( float error0, float error2, float *central_error );    //central difference   
float  getIntegral( float *Error, int n4_time, float iDError );    //central difference   

//--------------------------------------------//
// main module                                //
//--------------------------------------------//

int main()
{

	float x[ARRAY_LEN];       // Reference input  
	float Error[ARRAY_LEN];   // error for P  
	float iDError =0; // error for I 
	float centralDError; // errors for D (central difference)   
	float bufferError, errorPrevious;   
	//float disVePrevious;  

	float MicroStep;      //angle 

	float K_p;      //proportional controller gain  
	float K_i;      //integral controller gain  
	float K_d;      //derivative controller gain   

	float sumPID;                   //sum PID control action  
	float speedUnitTime; 

	int   upperBound;               //total number of loops for control action 
	int   integralBound;            //total number of erro points for integration controller   
	int   freqPwm;                  //PWM frequency 
	int   dirPwm;                   //PWM frequency 

	float angVe[ARRAY_LEN];         //vehicle angle 
	float disVe[ARRAY_LEN];         //lateral displacement basd angle and speed  


	float h[ARRAY_LEN];             //sensor output (sensing from contl(.))   
	float hPrime[ARRAY_LEN];        //convolution output of h(n)  

	int iindex; 

	//-----------------------------------------
	//  Generate testing data
	//-----------------------------------------

	open_buzzer();
	int ret = open_adc();
	printf("Return value from ADC Open %d\n ",ret);
	while(1){
		int choice = -1;
		printf("Select the choice for the operation\n");
		printf("1. Sine function\n");
		printf("2. Random function\n");
		printf("3. ADC function\n");
		printf("4. Constant input of 1\n");
		scanf("%d", &choice);
		printf("Enter the values for Kp, Ki and Kd\n");
		scanf("%d",&K_p);
		scanf("%d",&K_i);
		scanf("%d",&K_d);

		switch(choice)
		{
		int index = 0;
		int freq = 1500;

		int points_for_random = 0;

		case 1: //Sine function goes here.


			printf("kp ki kd %f %f %f \n", K_p, K_i, K_d);
			printf("Please Enter the total number of points for PID testing\n");
			printf("The maximum points should be less than 50K for this program :\n");
			scanf("%d", &upperBound);
			printf("The points for PID computation  = %4d \n", upperBound);

			integralBound = 4;   //time interval for integration controller    

			speedUnitTime = speedVe/(60.0*60.0); // meter per second 

			for(iindex = 0; iindex <=2 ; iindex++) 
			{
				float sin_val = sin((iindex % 60) * (3.1415/30));
				x[iindex] = sin_val;  // generate test data points  
				//x[iindex] = sin(iindex/(20)*pi);  // generate test data points  
				//-----------------------------------------------------------
				// check if initial start for n < 2, where n = 0, 1, 2, ... 
				// when n < 2, central difference can not be computed due to 
				// boundary conditions  
				//-----------------------------------------------------------
				disVe[iindex] = 0;  
				hPrime[iindex] = disVe[iindex];  

				hPrime[iindex+1] = hPrime[iindex]; // to take care of the first hPrime for index = 3   
				// for the computation for the rest of the points 
				// for PID control  
				bufferError = errorPrevious;//Error[iindex]; 

				if(iindex > 1)
					getDerive( Error[iindex-2], bufferError, &centralDError );    //central difference   
				//printf( "Main: Central Difference x = %4f\n", centralDError);   

				// find integral of error
				iDError = 0;     //at inital condition or near initial condition, set it 0  

				// find sume of PID 
				sumPID = K_p * errorPrevious + K_i * iDError + K_d * centralDError;  
				printf( "sumPID = %4f\n", sumPID);   

				if (sumPID < -1000)
				{
					sumPID = -1000;
				}
				if (sumPID > 1000)
				{
					sumPID = 1000;
				}

				// find PWM output 
				freqPwm = f_pwm;     // always at this version 
				//angVe[iindex] = (microStep8 * pi / 180) * sumPID ; // acutation to motor, angle to radius   
				angVe[iindex] = microStep8 * pi / 180 ; // acutation to motor, angle to radius   
				//-------acuate motor-------------------- 

				//send direction information to GPIO output here
				//add ARM GPIO output command  
				printf( "Error >= 0\n");   

				if (errorPrevious < 0) {
					dirPwm = -1; 
					//send direction information to GPIO output here
					//add ARM GPIO output command  
					angVe[iindex] = -1* angVe[iindex];  
					printf( "Error < 0\n");   
				}    

				disVe[iindex] = sumPID * speedUnitTime * angVe[iindex];   // for small angle, sin(angle) = angle  
				// find sensor output of the displacement 
				if( sumPID < 0) 
				{ 
					if(iindex > 0)
						h[iindex] = h[iindex-1] - disVe[iindex];     // sensing at ABSOLUTE coordinate system   
				}
				else
				{ 
					if(iindex > 0)
						h[iindex] = h[iindex-1] + disVe[iindex];     // sensing at ABSOLUTE coordinate system   
				}


				// simualate random noise additive to h(n) 
				// to be added later 

				// Low Pass Filtering operation via 1D convolution 
				// to find hPrime output  
				// to be added later 

				hPrime[iindex] = h[iindex];  
				Error[iindex] = x[iindex] - hPrime[iindex]; 
				errorPrevious = Error[iindex];  
				printf( "%4d : x= %4f h'= %4f error= %4f cDError= %4f sumpid=%4f iDError= %4f disve=%4f angve=%4f\n", iindex, x[iindex], hPrime[iindex], 		errorPrevious,centralDError, sumPID, iDError, disVe[iindex], angVe[iindex]);   
			}//for loop  


			for(iindex = 3; iindex <=upperBound ; iindex++) 
			{
				float radian = (3.1415 * iindex / 180);
				printf("Radian value %f\n",radian);
				float sin_val = sin((iindex % 60) * (3.1415/30));
				x[iindex] = sin_val;  // generate test data points  
				printf("Sine value %f\n",sin_val);
				bufferError = errorPrevious;//Error[iindex];

				getDerive( Error[iindex-2], bufferError, &centralDError );    //central difference   

				// find integral of error
				if(iindex >= integralBound ) 
				{ 
					iDError = 0;
					iDError = getIntegral( &Error, iindex, iDError );      
				} 

				// find sume of PID 
				sumPID = K_p * errorPrevious + K_i * iDError + K_d * centralDError;  

				if(sumPID > 0) 
					set_buzzer_freq(sumPID);

				if (sumPID < -1000)
				{
					sumPID = -1000;
				}
				if (sumPID > 1000)
				{
					sumPID = 1000;
				}


				// find PWM output 
				freqPwm = f_pwm;     // always at this version 
				//angVe[iindex] = (microStep8 * pi / 180) * sumPID ; // acutation to motor, angle to radius   
				angVe[iindex] = microStep8 * pi / 180 ; // acutation to motor, angle to radius   
				//-------acuate motor-------------------- 

				if (errorPrevious >= 0)  
				{ 
					dirPwm = 1; 
					//send direction information to GPIO output here
					//add ARM GPIO output command  
				} 
				else if (errorPrevious < 0) 
				{
					dirPwm = -1; 
					//send direction information to GPIO output here
					//add ARM GPIO output command  
					angVe[iindex] = -1* angVe[iindex];  
				}    

				disVe[iindex] = sumPID * speedUnitTime * angVe[iindex];   // for small angle, sin(angle) = angle  

				// find sensor output of the displacement 
				if( sumPID < 0) 
				{ 
					h[iindex] = h[iindex-1] - disVe[iindex];     // sensing at ABSOLUTE coordinate system   
				}
				else
				{ 
					h[iindex] = h[iindex-1] + disVe[iindex];     // sensing at ABSOLUTE coordinate system   
				}

				// simualate random noise additive to h(n) 
				// to be added later 

				// Low Pass Filtering operation via 1D convolution 
				// to find hPrime output  
				// to be added later 

				hPrime[iindex] = h[iindex];  
				Error[iindex] = x[iindex] - hPrime[iindex]; 
				errorPrevious = Error[iindex];  
				printf( "%4d : x= %4f h'= %4f error= %4f cDError=%4f sum_pid=%4f iDError=%4f disve=%4f angve=%4f\n", iindex, x[iindex], hPrime[iindex], 		errorPrevious,centralDError,sumPID, iDError, disVe[iindex], angVe[iindex]);   
				//sleep(2);0
			}//for loop  
			stop_buzzer();	

			break;

		case 2://random function goes here.
			printf("Enter the number of points to go over the random function\n");
			scanf("%d",&points_for_random);

			for(index = 0; index < points_for_random; index++)
			{
				int rand_val = rand();
				printf("Rand val %d\n",rand_val);
				//        	rand_val/=10;
				rand_val = rand_val % 10;
				rand_val /= 100;
				int set_freq = rand_val + freq;
				printf("Set freq %d \n",set_freq);
				set_buzzer_freq(set_freq);		
			}


			stop_buzzer();
			break;

		case 3: //Case with ADC as input to PID controller

			printf("Please Enter the total number of points for PID testing\n");
			printf("The maximum points should be less than 50K for this program :\n");
			scanf("%d", &upperBound);
			printf("The points for PID computation  = %4d \n", upperBound);

			integralBound = 4;   //time interval for integration controller    
			//printf( "Integral time  = %4d\n", integralBound);   

			speedUnitTime = speedVe/(60.0*60.0); // meter per second 
			//K_p = 1;//310; 
			//K_i = 1;//10; 
			//K_d = 1;//430; 


			for(iindex = 0; iindex <=2 ; iindex++) 
			{
				x[iindex] = 1;  // generate test data points  
				//x[iindex] = sin(iindex/(20)*pi);  // generate test data points  
				//-----------------------------------------------------------
				// check if initial start for n < 2, where n = 0, 1, 2, ... 
				// when n < 2, central difference can not be computed due to 
				// boundary conditions  
				//-----------------------------------------------------------
				disVe[iindex] = 0;  
				hPrime[iindex] = disVe[iindex];  

				hPrime[iindex+1] = hPrime[iindex]; // to take care of the first hPrime for index = 3   
				// for the computation for the rest of the points 
				// for PID control  
				bufferError = errorPrevious;//Error[iindex]; 
				if(iindex > 1)
					getDerive( Error[iindex-2], bufferError, &centralDError );    //central difference   

				// find integral of error
				iDError = 0;     //at inital condition or near initial condition, set it 0  

				// find sume of PID 
				sumPID = K_p * errorPrevious + K_i * iDError + K_d * centralDError;  
				printf( "sumPID = %4f\n", sumPID);   

				if (sumPID < -1000)
				{
					sumPID = -1000;
				}
				if (sumPID > 1000)
				{
					sumPID = 1000;
				}

				// find PWM output 
				freqPwm = f_pwm;     // always at this version 
				//angVe[iindex] = (microStep8 * pi / 180) * sumPID ; // acutation to motor, angle to radius   
				angVe[iindex] = microStep8 * pi / 180 ; // acutation to motor, angle to radius   
				//-------acuate motor-------------------- 

				//send direction information to GPIO output here
				//add ARM GPIO output command  
				printf( "Error >= 0\n");   
				if (errorPrevious < 0) {
					dirPwm = -1; 
					//send direction information to GPIO output here
					//add ARM GPIO output command  
					angVe[iindex] = -1* angVe[iindex];  
					printf( "Error < 0\n");   
				}    

				disVe[iindex] = sumPID * speedUnitTime * angVe[iindex];   // for small angle, sin(angle) = angle  
				// find sensor output of the displacement 
				if( sumPID < 0) 
				{ 
					if(iindex > 0)
						h[iindex] = h[iindex-1] - disVe[iindex];     // sensing at ABSOLUTE coordinate system   
				}
				else
				{ 
					if(iindex > 0)
						h[iindex] = h[iindex-1] + disVe[iindex];     // sensing at ABSOLUTE coordinate system   
				}


				// simualate random noise additive to h(n) 
				// to be added later 

				// Low Pass Filtering operation via 1D convolution 
				// to find hPrime output  
				// to be added later 

				hPrime[iindex] = h[iindex];  
				Error[iindex] = x[iindex] - hPrime[iindex]; 
				errorPrevious = Error[iindex];  
				printf( "%4d : x= %4f h'= %4f error= %4f cDError= %4f sumpid=%4f iDError= %4f disve=%4f angve=%4f\n", iindex, x[iindex], hPrime[iindex], 		errorPrevious,centralDError, sumPID, iDError, disVe[iindex], angVe[iindex]);   
			}//for loop  


			for(iindex = 3; iindex <=upperBound ; iindex++) 
			{
				x[iindex] = read_adc();  // generate test data points  
				bufferError = errorPrevious;//Error[iindex];

				getDerive( Error[iindex-2], bufferError, &centralDError );    //central difference   


				// find integral of error
				if(iindex >= integralBound ) { 
					iDError = 0;
					iDError = getIntegral( &Error, iindex, iDError );      
				} 

				// find sume of PID 
				sumPID = K_p * errorPrevious + K_i * iDError + K_d * centralDError;  
				//printf( "sumPID = %4f\n", sumPID);  
				if(sumPID > 0) 
					set_buzzer_freq(sumPID);

				if (sumPID < -1000)
				{
					sumPID = -1000;
				}
				if (sumPID > 1000)
				{
					sumPID = 1000;
				}


				// find PWM output 
				freqPwm = f_pwm;     // always at this version 
				//angVe[iindex] = (microStep8 * pi / 180) * sumPID ; // acutation to motor, angle to radius   
				angVe[iindex] = microStep8 * pi / 180 ; // acutation to motor, angle to radius   
				//-------acuate motor-------------------- 

				if (errorPrevious >= 0)  { 
					dirPwm = 1; 
					//send direction information to GPIO output here
					//add ARM GPIO output command  
					//printf( "Error >= 0\n");   
				} 
				else if (errorPrevious < 0) {
					//else if ((errorPrevious < 0) | (sumPID < 0)) {
					dirPwm = -1; 
					//send direction information to GPIO output here
					//add ARM GPIO output command  
					angVe[iindex] = -1* angVe[iindex];  
					//printf( "Error < 0\n");   
				}    

				disVe[iindex] = sumPID * speedUnitTime * angVe[iindex];   // for small angle, sin(angle) = angle  

				// find sensor output of the displacement 
				if( sumPID < 0) 
				{ 
					h[iindex] = h[iindex-1] - disVe[iindex];     // sensing at ABSOLUTE coordinate system   
				}
				else
				{ 
					h[iindex] = h[iindex-1] + disVe[iindex];     // sensing at ABSOLUTE coordinate system   
				}

				// simualate random noise additive to h(n) 
				// to be added later 

				// Low Pass Filtering operation via 1D convolution 
				// to find hPrime output  
				// to be added later 

				hPrime[iindex] = h[iindex];  
				Error[iindex] = x[iindex] - hPrime[iindex]; 
				errorPrevious = Error[iindex];  
				printf( "%4d : x= %4f h'= %4f error= %4f cDError=%4f sum_pid=%4f iDError=%4f disve=%4f angve=%4f\n", iindex, x[iindex], hPrime[iindex], 		errorPrevious,centralDError,sumPID, iDError, disVe[iindex], angVe[iindex]);   
				sleep(2);
			}//for loop  
			stop_buzzer();	

			break;


		case 4: //Function for 1 input goes here.
			printf("kp ki kd %f %f %f \n", K_p, K_i, K_d);

			printf("Please Enter the total number of points for PID testing\n");
			printf("The maximum points should be less than 50K for this program :\n");
			scanf("%d", &upperBound);
			printf("The points for PID computation  = %4d \n", upperBound);

			integralBound = 4;   //time interval for integration controller    
			//printf( "Integral time  = %4d\n", integralBound);   

			speedUnitTime = speedVe/(60.0*60.0); // meter per second 
			//K_p = 1;//310; 
			//K_i = 1;//10; 
			//K_d = 1;//430; 


			for(iindex = 0; iindex <=2 ; iindex++) 
			{
				x[iindex] = 1;  // generate test data points  
				//x[iindex] = sin(iindex/(20)*pi);  // generate test data points  
				//-----------------------------------------------------------
				// check if initial start for n < 2, where n = 0, 1, 2, ... 
				// when n < 2, central difference can not be computed due to 
				// boundary conditions  
				//-----------------------------------------------------------
				disVe[iindex] = 0;  
				hPrime[iindex] = disVe[iindex];  

				hPrime[iindex+1] = hPrime[iindex]; // to take care of the first hPrime for index = 3   
				// for the computation for the rest of the points 
				// for PID control  
				bufferError = errorPrevious;//Error[iindex]; 
				if(iindex > 2)
					getDerive( Error[iindex-3], bufferError, &centralDError );    //central difference   
				//printf( "Main: Central Difference x = %4f\n", centralDError);   

				// find integral of error
				iDError = 0;     //at inital condition or near initial condition, set it 0  

				// find sume of PID 
				sumPID = K_p * errorPrevious + K_i * iDError + K_d * centralDError;  
				//printf( "sumPID = %4f\n", sumPID);   

				if (sumPID < -1000)
				{
					sumPID = -1000;
				}
				if (sumPID > 1000)
				{
					sumPID = 1000;
				}

				// find PWM output 
				freqPwm = f_pwm;     // always at this version 
				//angVe[iindex] = (microStep8 * pi / 180) * sumPID ; // acutation to motor, angle to radius   
				angVe[iindex] = microStep8 * pi / 180 ; // acutation to motor, angle to radius   
				//-------acuate motor-------------------- 

				//send direction information to GPIO output here
				//add ARM GPIO output command  
				//printf( "Error >= 0\n");   
				if (errorPrevious < 0) {
					dirPwm = -1; 
					//send direction information to GPIO output here
					//add ARM GPIO output command  
					angVe[iindex] = -1* angVe[iindex];  
					//printf( "Error < 0\n");   
				}    

				// printf( "Angle = %4f\n", angVe[iindex]);   
				// find lateral displacement i
				// disVe[iindex] = speedVe * sin(angVe[iindex]);   // 
				//disVe[iindex] = speedUnitTime * angVe[iindex];   // for small angle, sin(angle) = angle  
				disVe[iindex] = sumPID * speedUnitTime * angVe[iindex];   // for small angle, sin(angle) = angle  
				// find sensor output of the displacement 
				if( sumPID < 0) 
				{ 
					if(iindex > 0)
						h[iindex] = h[iindex-1] - disVe[iindex];     // sensing at ABSOLUTE coordinate system   
				}
				else
				{ 
					if(iindex > 0)
						h[iindex] = h[iindex-1] + disVe[iindex];     // sensing at ABSOLUTE coordinate system   
				}


				// simualate random noise additive to h(n) 
				// to be added later 

				// Low Pass Filtering operation via 1D convolution 
				// to find hPrime output  
				// to be added later 

				hPrime[iindex] = h[iindex];  
				Error[iindex] = x[iindex] - hPrime[iindex]; 
				errorPrevious = Error[iindex];  
				printf( "%4d : x= %4f h'= %4f error= %4f cDError= %4f sumpid=%4f iDError= %4f disve=%4f angve=%4f\n", iindex, x[iindex], hPrime[iindex], 		errorPrevious,centralDError, sumPID, iDError, disVe[iindex], angVe[iindex]);   
			}//for loop  


			for(iindex = 3; iindex <=upperBound ; iindex++) 
			{
				x[iindex] = 1;  // generate test data points  
				//x[iindex] = sin((iindex/20)*pi);  // generate test data points  
				//Error[iindex] = x[iindex] - hPrime[iindex]; 
				//printf( "Top %4f %4f\n", hPrime[iindex], Error[iindex]);   
				// find derivative of error
				//getDerive(x[iindex-2], x[iindex], &centralDError);    // only turn on when testing derivative on x(n) data  
				bufferError = errorPrevious;//Error[iindex];

				getDerive( Error[iindex-3], bufferError, &centralDError );    //central difference   
				//printf( "Main: Central Difference x = %4f\n", centralDError);   


				// find integral of error
				if(iindex >= integralBound ) { 
					iDError = 0;
					iDError = getIntegral( &Error, iindex, iDError );      
					//printf( "Main: Integral Error = %4f\n", iDError);   
				} 

				// find sume of PID 
				sumPID = K_p * errorPrevious + K_i * iDError + K_d * centralDError;  
				//printf( "sumPID = %4f\n", sumPID);  
				if(sumPID > 0) 
					set_buzzer_freq(sumPID);

				if (sumPID < -1000)
				{
					sumPID = -1000;
				}
				if (sumPID > 1000)
				{
					sumPID = 1000;
				}


				// find PWM output 
				freqPwm = f_pwm;     // always at this version 
				//angVe[iindex] = (microStep8 * pi / 180) * sumPID ; // acutation to motor, angle to radius   
				angVe[iindex] = microStep8 * pi / 180 ; // acutation to motor, angle to radius   
				//-------acuate motor-------------------- 

				if (errorPrevious >= 0)  { 
					dirPwm = 1; 
					//send direction information to GPIO output here
					//add ARM GPIO output command  
					//printf( "Error >= 0\n");   
				} 
				else if (errorPrevious < 0) {
					//else if ((errorPrevious < 0) | (sumPID < 0)) {
					dirPwm = -1; 
					//send direction information to GPIO output here
					//add ARM GPIO output command  
					angVe[iindex] = -1* angVe[iindex];  
					//printf( "Error < 0\n");   
				}    

				// find lateral displacement i
				// disVe[iindex] = speedVe * sin(angVe[iindex]);   // 
				// disVe[iindex] = speedUnitTime * angVe[iindex];   // for small angle, sin(angle) = angle  
				disVe[iindex] = sumPID * speedUnitTime * angVe[iindex];   // for small angle, sin(angle) = angle  

				// find sensor output of the displacement 
				if( sumPID < 0) 
				{ 
					h[iindex] = h[iindex-1] - disVe[iindex];     // sensing at ABSOLUTE coordinate system   
				}
				else
				{ 
					h[iindex] = h[iindex-1] + disVe[iindex];     // sensing at ABSOLUTE coordinate system   
				}

				// simualate random noise additive to h(n) 
				// to be added later 

				// Low Pass Filtering operation via 1D convolution 
				// to find hPrime output  
				// to be added later 

				hPrime[iindex] = h[iindex];  
				Error[iindex] = x[iindex] - hPrime[iindex]; 
				errorPrevious = Error[iindex];  
				printf( "%4d : x= %4f h'= %4f error= %4f cDError=%4f sum_pid=%4f iDError=%4f disVe=%4f angve=%4f\n", iindex, x[iindex], hPrime[iindex], 		errorPrevious,centralDError,sumPID, iDError,disVe[iindex], angVe[iindex]);   
				sleep(2);
			}//for loop  
			stop_buzzer();
			break;//case for input 1

		}


	}
	return 0;
}  

/*-----------------------------------------------
	Compute Derivatives 
-------------------------------------------------*/ 
void  getDerive(float error0, float error2, float *c_difference)     //central difference   
{
	*c_difference = (error2 - error0)/2; // central difference   
	return;
}  

/*---------------------------------------------
 Integral
----------------------------------------------*/

float  getIntegral( float *Error, int numTime, float iDError )     //integral
{
	int i; 
	int index = numTime - 4;
	for (i=index; i <= numTime; i++)
	{ 
		if(i >= 0)
			iDError = iDError + Error[i]*Error[i];
	}  
	
	return iDError;
}   


/*-----------------------------------------------
PWM Code
-------------------------------------------------*/

int pwm_fd = -1;
void close_buzzer(void);
void open_buzzer(void)
{
	pwm_fd = open("/dev/pwm", 0);
	printf("Buzzer opened with fd %d !!\n ",pwm_fd);
	if (pwm_fd < 0) {
		perror("open pwm_buzzer device");
		exit(1);
	}

	// any function exit call will stop the buzzer
	atexit(close_buzzer);

}

void close_buzzer(void)
{
	if (pwm_fd >= 0) {
		ioctl(pwm_fd, PWM_IOCTL_STOP);
		if (ioctl(pwm_fd, 2) < 0) {
			perror("ioctl 2:");
		}
		close(pwm_fd);
		pwm_fd = -1;
	}
}

void set_buzzer_freq(float freq)
{

	int set_freq = (int)freq;
	// this IOCTL command is the key to set frequency
	if(set_freq > -3500 & set_freq < -3000)
		set_freq = 1500;
	if(set_freq > -3000 & set_freq < -2500)
		set_freq = 1700;
	if(set_freq > -2500 & set_freq < -2000)
		set_freq = 1900;
	if(set_freq > -2000 & set_freq < -1500)
		set_freq = 2100;
	if(set_freq > -1500 & set_freq < -1000)
		set_freq = 2300;
	if(set_freq > -1000 & set_freq < -500)
		set_freq = 2500;
	if(set_freq > -500 & set_freq < -1)
		set_freq = 2700;
	if(set_freq < -3500)
		set_freq = 5000;
	if(set_freq < 20 & set_freq > 0)
		set_freq = 2000;
	set_freq *= 100;
	set_freq+=3000;
	//printf("Setting freq %d\n",set_freq);
	if(set_freq == 0)
		set_freq = set_freq+100;
	int ret = ioctl(pwm_fd, PWM_IOCTL_SET_FREQ, set_freq);
	if(ret < 0) {
		perror("set the frequency of the buzzer");
		exit(1);
	}
}
void stop_buzzer(void)
{
	int ret = ioctl(pwm_fd, PWM_IOCTL_STOP);
	if(ret < 0) {
		perror("stop the buzzer");
		exit(1);
	}
	if (ioctl(pwm_fd, 2) < 0) {
		perror("ioctl 2:");
	}
}

/*------------------------------------
ADC Module
--------------------------------------*/


int adc_fd = -1;
int open_adc()
{
	adc_fd = open("/dev/newadc", 0);
	if (adc_fd < 0) {
		perror("open ADC device:");
		return 1;
	}
	printf("Opened ADC\n");
	return 0;
}

int read_adc()
{
	char buffer[30];
	int value = -1;
	int len = read(adc_fd, buffer, sizeof buffer -1);
	if (len > 0) {
		buffer[len] = '\0';
		value = -1;
		sscanf(buffer, "%d", &value);
		printf("Converted AD Value : %d\n", value);
	} else {
		perror("read ADC device:");
		return 1;
	}
	return value;
}

float cal_lateral_error(float speed, float angular_error)
{
	return sin(3.1415 * angular_error / 180) * speed;
}

float* make_kernel(int radius, float sigma)
{

	float kernel[radius * radius];
	float sum = 0;
	int y = 0;
	int x = 0;

	for (y = 0; y < radius; y++)
	{
		for (x = 0; x < radius; x++)
		{
			int off = y * radius + x;
			int xx = x - radius / 2;
			int yy = y - radius / 2;
			//printf("Array value %f ",(float)exp(-(xx*xx + yy*yy)/(2*sigma*sigma)));
			kernel[off] = (float)exp(-(xx*xx + yy*yy)/(2*sigma*sigma));

			sum += kernel[off];
		}
	}

	int i = 0;
	int length = (sizeof(kernel)/sizeof(kernel[0]));

	for (i = 0; i < length; i++)
	{
		kernel[i] /= sum;
	}


	x = 0;
	y = 0;
	printf("----Gaussian Kernel------\n");
	for (x = 0; x < radius; x++)
	{
		for (y = 0; y < radius; y++)
		{
			printf("%f ",kernel[y * radius + x]);
		}
		printf("\n");
	}

	return kernel;

}

void Laplace_Kernel(int n, float sigma)
{
	printf("\n");
	printf("------Laplace Kernel------\n");
	double kernel[n * n];

	int i;
	int off;
	int col;
	int x;
	int y;
	double sum = 0;
	int max = n*n;

	for(i = 0; i < max; i++)
	{
		off = (int)(i / n);
		col = i - (n * off);
		y = ((int)(n / 2)) - off;
		x = col - ((int)(n / 2));
		kernel[i] = ( (x * x + y * y - 2 * sigma * sigma)/(sigma * sigma * sigma * sigma) )
				* exp(-(x * x + y * y)/(2 * sigma * sigma));
	}

	x = 0;
	y = 0;

	for (x = 0; x < n; x++)
	{
		for (y = 0; y < n; y++)
		{
			printf("%f ",kernel[y * n + x]);
		}
		printf("\n");
	}
}


