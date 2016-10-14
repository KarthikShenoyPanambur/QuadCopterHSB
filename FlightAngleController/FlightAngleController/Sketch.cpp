
#include <Wire.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <HMC5883L.h>
#include <TimerThree.h>

#define MOTOR_DRIVE_PERIOD 102400  //In microseconds
#define PWM_PIN 11


struct movingAvgFilter
{	
	float newDataCoeff,oldDataCoeff;
	int16_t Sum_roll,Sum_pitch,Sum_yaw;	
	int16_t avgRoll,avgPitch,avgYaw;
};


struct    // mpuSensor
{
	// Set the FreeSixIMU object
	FreeSixIMU sixDOF = FreeSixIMU();
	HMC5883L compass;
	
	// Record any errors that may occur in the compass.
	int error = 0;
	bool SensorReadFlag;
	float accelr_x,accelr_y,accelr_z;
	float gyro_x,gyro_y,gyro_z;
	float prev_gyro_x,prev_gyro_y,prev_gyro_z;
	float magn_x,magn_y,magn_z;
	float roll,pitch,yaw;
	float gyro_x_diff,gyro_y_diff,gyro_z_diff;
	movingAvgFilter mAvgFilter;
}mpuSensor;

#define GYRO_TIME_DIFF_COUNT 1

void movingAverageFilter(float newRoll,float newPitch,float newYaw,movingAvgFilter* avgFilter);
void movingAverageFilter(float newRoll,float newPitch,float newYaw,movingAvgFilter* avgFilter)
{
	avgFilter->Sum_roll = (((avgFilter->Sum_roll*avgFilter->oldDataCoeff)) + (avgFilter->newDataCoeff*  newRoll))/(avgFilter->oldDataCoeff+avgFilter->newDataCoeff);
	avgFilter->Sum_pitch = (((avgFilter->Sum_pitch*avgFilter->oldDataCoeff)) + (avgFilter->newDataCoeff * newPitch))/(avgFilter->oldDataCoeff+avgFilter->newDataCoeff);
	avgFilter->Sum_yaw = (((avgFilter->Sum_yaw *avgFilter->oldDataCoeff))+ (avgFilter->newDataCoeff * newYaw))/(avgFilter->oldDataCoeff+avgFilter->newDataCoeff);
	
	//Averaging the values.
	avgFilter->avgRoll = (int16_t)(avgFilter->Sum_roll);             // - sensorOffset.AcclOffset_roll;//(avgFilter->filterCoeff[1]+1);            //FILTER_LENGHT;
	avgFilter->avgPitch = (int16_t)(avgFilter->Sum_pitch);          // - sensorOffset.AcclOffset_pitch;//(avgFilter->filterCoeff[1]+1);           //FILTER_LENGHT;
	avgFilter->avgYaw = (int16_t)(avgFilter->Sum_yaw);
	
}

void printRollPitchYaw(float roll, float pitch,float yaw);
void printRollPitchYaw(float roll, float pitch,float yaw)
{
	Serial.print("|| \t rot abt x = \t ");
	Serial.print(roll);
	Serial.print("|| \t rot abt y =  \t ");
	Serial.print(pitch);
	Serial.print("|| \t rot abt z = \t ");
	Serial.println(yaw);
}


void tiltAngleCalculator(float *roll,float *pitch,float *yaw);
void tiltAngleCalculator(float *roll,float *pitch,float *yaw)
{
	float accl[3];
	float heading;
	mpuSensor.sixDOF.getValues(accl);
	mpuSensor.accelr_x=accl[0];
	mpuSensor.accelr_y=accl[1];
	mpuSensor.accelr_z=accl[2];
	
	mpuSensor.sixDOF.gyro.readGyro(&mpuSensor.gyro_x,&mpuSensor.gyro_y,&mpuSensor.gyro_z);	
	mpuSensor.gyro_x_diff=mpuSensor.prev_gyro_x - mpuSensor.gyro_x;
	mpuSensor.gyro_y_diff=mpuSensor.prev_gyro_y - mpuSensor.gyro_y;
	mpuSensor.gyro_z_diff=mpuSensor.prev_gyro_z - mpuSensor.gyro_z;
	
	// Retrive the raw values from the compass (not scaled).
	MagnetometerRaw raw = mpuSensor.compass.ReadRawAxis();
	// Retrived the scaled values from the compass (scaled to the configured scale).
	MagnetometerScaled scaled = mpuSensor.compass.ReadScaledAxis();
	// Values are accessed like so:
	int MilliGauss_OnThe_XAxis = scaled.XAxis;// (or YAxis, or ZAxis)
	// Calculate heading when the magnetometer is level, then correct for signs of axis.
	heading = atan2(scaled.YAxis, scaled.XAxis);
	//float declinationAngle = 0.0457;
	//heading += declinationAngle;	
	// Correct for when signs are reversed.
	if(heading < 0)
	heading += 2*PI;
	// Check for wrap due to addition of declination.
	if(heading > 2*PI)
	heading -= 2*PI;
	// Convert radians to degrees for readability.
	heading = heading * 180/M_PI;
	//*roll = (atan2(mpuSensor.accelr_y , mpuSensor.accelr_z) * 57.3);
	
	*roll=*roll+((-mpuSensor.gyro_x_diff/32.8f)*(0.5f));
	*roll =0.8* (*roll) + (1.0f-0.8)*-(atan2(mpuSensor.accelr_y , mpuSensor.accelr_z) * 57.3);
	//*pitch = -(atan2((- accl_x) , sqrt(accl_y * accl_y + accl_z * accl_z)) * 57.3);
	*pitch=*pitch+((-mpuSensor.gyro_y_diff/32.8f)*(0.5f));
	*pitch =0.8* (*pitch) + (1.0f-0.8)*-(atan2((- mpuSensor.accelr_x) , sqrt(mpuSensor.accelr_y * mpuSensor.accelr_y + mpuSensor.accelr_z * mpuSensor.accelr_z)) * 57.3);
	*yaw= heading;
}



void mpuInit();
void mpuInit()
{
	mpuSensor.SensorReadFlag=false;
	mpuSensor.sixDOF.init(); //init the Acc and Gyro
	mpuSensor.compass = HMC5883L(); // init HMC5883
	mpuSensor.error = mpuSensor.compass.SetScale(1.3); // Set the scale of the compass.
	mpuSensor.error = mpuSensor.compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
	if(mpuSensor.error != 0) // If there is an error, print it out.
	Serial.println(mpuSensor.compass.GetErrorText(mpuSensor.error));
	//bmp085Calibration(); // init barometric pressure sensor
	
	mpuSensor.mAvgFilter.newDataCoeff=1;
	mpuSensor.mAvgFilter.oldDataCoeff=1;
}


void mpuSensorReadCallBack(void);
void mpuSensorReadCallBack(void)
{
	mpuSensor.SensorReadFlag=true;
}

enum states{IDLE,MEASURE,CONTROL,DRIVE};
struct stateMachine
{
	states currentState,nextState;
	bool startQuadCopter;
};
struct stateMachine fsm;


struct pulseWidthModulation
{
	int16_t dutyCycle;
	int16_t pwmPin;
};
struct pulseWidthModulation pwmMotor_1;


//global variables



void setup()
{
	Serial.begin(9600);	
	Wire.begin();
	mpuInit();
	Timer3.initialize(500);
	Timer3.attachInterrupt(mpuSensorReadCallBack);
	
	fsm.startQuadCopter=true;
	fsm.currentState=IDLE;
	fsm.nextState=IDLE;
	pwmMotor_1.dutyCycle=512;	
}





void loop()
{
	Timer3.start();
	switch(fsm.currentState)
	{
		case IDLE:
					  //Serial.println("\n+++++++++++++++++++++++++++++++\n");
					  //Serial.println("IDLE state");
					  if(fsm.startQuadCopter==true)
					  {
						fsm.nextState=MEASURE;
					  }
					  else
						fsm.nextState=IDLE;
						
					  break;
		case MEASURE:
					 movingAverageFilter(mpuSensor.roll,mpuSensor.pitch,mpuSensor.yaw,&mpuSensor.mAvgFilter);
					 printRollPitchYaw(mpuSensor.mAvgFilter.avgRoll,mpuSensor.mAvgFilter.avgPitch,mpuSensor.mAvgFilter.avgYaw);	
					 fsm.nextState=CONTROL;
					 break;
		case CONTROL:
					 //Serial.println("\n CONTROL state");
					 //PID.ControlAlgo(mpuSensor.mAvgFilter.avgRoll,mpuSensor.mAvgFilter.avgPitch,mpuSensor.mAvgFilter.avgYaw);
					 fsm.nextState=DRIVE;
					 break;
		case DRIVE:
				     //Serial.println("DRIVE state");
		            // pwmGenerator.setPwmDuty(PWM_PIN,pulseWidthModulation.dutyCycle);
				     fsm.nextState=IDLE;
				     break;
				
	}fsm.currentState=fsm.nextState;	
	if(mpuSensor.SensorReadFlag=true)
		tiltAngleCalculator(&mpuSensor.roll,&mpuSensor.pitch,&mpuSensor.yaw);
	
	//temperature = bmp085GetTemperature(bmp085ReadUT());
	//pressure = bmp085GetPressure(bmp085ReadUP());
}






