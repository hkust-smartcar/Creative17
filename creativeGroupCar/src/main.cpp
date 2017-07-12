/*
 * main.cpp
 *
 * Author: Gordon
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#define _GLIBCXX_USE_C99 1

#include <cassert>
#include <cstring>
#include <string>
#include <vector>
#include <array>
#include <queue>
#include <cstdint>
#include "Coor.h"
#include "PID.h"
#include "Sr04.h"
#include "Qmc5883.h"
#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include <libsc/led.h>
#include <libsc/button.h>
#include <libbase/k60/gpio.h>
#include <libbase/k60/pin.h>
//#include <libsc/k60/ov7725.h>
//#include <libsc/k60/ov7725_configurator.h>
#include <libsc/k60/uart_device.h>
#include <libsc/k60/jy_mcu_bt_106.h>
#include <libsc/futaba_s3010.h>
#include <libsc/dir_motor.h>
#include <libsc/mpu6050.h>
#include <libsc/mini_lcd.h>
#include <libsc/joystick.h>
#include <libsc/st7735r.h>
#include <libsc/lcd_typewriter.h>
#include <libutil/math.h>
#include <libsc/button.h>
#include <libsc/us_100.h>
#include <libsc/encoder.h>
#include <libsc/dir_encoder.h>
#include <math.h>


namespace libbase
{
	namespace k60
	{

		Mcg::Config Mcg::GetMcgConfig()
		{
			Mcg::Config config;
			config.external_oscillator_khz = 50000;
			config.core_clock_khz = 200000;
			return config;
		}

	}
}

using namespace libsc;
using namespace libbase::k60;
using namespace libsc::k60;
using namespace libutil;

struct vec
{
    int32_t x, y, z;
    vec():x(0), y(0), z(0){}
    vec(int32_t X, int32_t Y, int32_t Z):x(X), y(Y), z(Z){}
    vec(const vec& v):x(v.x), y(v.y), z(v.z){}

    vec operator+(const vec& v)
    {
        return vec(x+v.x, y+v.y, z+v.z);
    }

    vec operator-(const vec& v)
    {
        return vec(x-v.x, y-v.y, z-v.z);
    }

    vec& operator=(const vec& v)
    {
    	x = v.x;
    	y = v.y;
    	z = v.z;
        return *this;
    }
    void print(JyMcuBt106* bP)
    {
    	char buffer[80];
    	sprintf(buffer, "<%d, %d, %d>", x, y, z);
    	bP->SendStr(buffer);
    	bP->SendStr("\n");
    }
};


#define CAM_W 80
#define CAM_H 60
//#define ENABLE_LCD
#define DISTANCE_CHECK 20
#define DATA_WIDTH 4
const float countConstant = 100000;
#define PI 3.14159265358979323846

#define SERVO_LEFT_LIMIT 1140
#define SERVO_CENTRE 850
#define SERVO_RIGHT_LIMIT 565

#define COUNT_FOR_ONE_METER 57400

#define DISTANCE_FOR_UNLOCK_SERVO 1700 / 2



//the centre received through bluetooth will be stored in the global variable centre
//so use centre to do process also
float carLocate();

uint32_t distanceSquare(const Coor& a, const Coor& b);

void boundAngle(float& angle);

void motorSetPower(int16_t pL, int16_t pR);

float updateCompassAngle();

void dodgeObstacle();

float angleTracking(Coor carT, Coor carH);


//k60::Ov7725::Config getCameraConfig();

UartDevice::Config getBluetoothConfig();
Mpu6050::Config getMpuConfig();
St7735r::Config getLcdConfig();
Joystick::Config getJoystickConfig(uint8_t _id);

//assume receive centre data in the following format
//sXXX,XXX,XXX,XXX,e the first Coor is beacon
bool bluetoothListenerOne(const Byte* data, const size_t size);
bool bluetoothListenerTwo(const Byte* data, const size_t size);



Led* ledP[4];
St7735r* lcdP;
JyMcuBt106* bluetoothP[2];
Joystick* joystickP;
LcdTypewriter* writerP;
Mpu6050* mpuP;
DirMotor* motorLP;
DirMotor* motorRP;
Qmc5883* compassP;
DirEncoder* encoderLP;
DirEncoder* encoderRP;
FutabaS3010* servoP;

Coor carH;
Coor carT;
Coor beacon;
std::vector<Coor> Obstacle;
bool startFlag = false;
float distance = 0.0f;

std::array<int32_t, 3> acc = {0, 0, 0};
std::array<int32_t, 3> gyo = {0, 0, 0};
std::vector<vec> accSample;

//for car angle
float carHeadingAngle;
float carAngleError; //positive need to turn left, negative need to turn right
float carAngleErrorPrevious;

//for receiving data form bluetooth
std::string messageFromDrone;
std::vector<int16_t> coorBuffer;

//servo control
float servoKP = 5;
float servoKD = 0;
uint16_t servoOutput;

//for ulrasonic sensor
uint32_t timeForUltraL = 0;
float rangeForUltraL = 0.0;
uint32_t timeForUltraR = 0;
float rangeForUltraR = 0.0;

//for dogging obstacle
int32_t distanceForLockServo = 0;
uint8_t countForLockServoL = 0;
uint8_t countForLockServoR = 0;
int32_t eReadingL = 0;
int32_t eReadingR = 0;

PID motorLControl;
PID motorRControl;

bool startTheCarProcess = false;
bool programEnd = false;

bool irOn = false;
bool lockServo = false;
bool moveBack = false;
bool moveForward = false;

int32_t accEncL = 0;
int32_t accEncR = 0;

float compassAngle;

int main(void)
{
	System::Init();



	//---------------------------------button
	Button::Config buttonC;
	buttonC.is_use_pull_resistor = true;
	buttonC.is_active_low = true;
	buttonC.id = 0;
	buttonC.listener_trigger = Button::Config::Trigger::kDown;
	buttonC.listener =  [](const uint8_t id)
						{
//							for(int i = 0; i < 3; i++)
//							{
//								char buffer[50];
//								sprintf(buffer, "%d: %d\t", i, acc[i]);
//								bluetoothP->SendStr(buffer);
//							}
						};

	//---------------------------------irLedSwitch
	Gpo::Config gpoC;
	gpoC.pin = libbase::k60::Pin::Name::kPta4;
	Gpo ir(gpoC);
	ir.Reset();

	//---------------------------------led
	Led::Config c;
	c.is_active_low = 1;
	c.id = 0;
	Led led0(c);
	c.id = 1;
	Led led1(c);
	c.id = 2;
	Led led2(c);
	c.id = 3;
	Led led3(c);
	ledP[0] = &led0;
	ledP[1] = &led1;
	ledP[2] = &led2;
	ledP[3] = &led3;

	//--------------------------------lcd
	St7735r lcd(getLcdConfig());
	lcdP = &lcd;
	LcdTypewriter::Config config;
	config.lcd = &lcd; // St7735r lcd from prev page
	LcdTypewriter writer(config);
	writerP = &writer;

	//--------------------------------bluetooth
	UartDevice::Config bluetoothC;
	bluetoothC.id = 0;
	bluetoothC.baud_rate = Uart::Config::BaudRate::k115200;
	bluetoothC.rx_isr = bluetoothListenerOne;
	JyMcuBt106 bt1(bluetoothC);
	bluetoothP[0] = &bt1;

	bluetoothC.id = 1;
	bluetoothC.rx_isr = bluetoothListenerTwo;
	JyMcuBt106 bt2(bluetoothC);
	bluetoothP[1] = &bt2;

	//--------------------------------mpu
	Mpu6050 mpu(getMpuConfig());
	mpuP = &mpu;

//	//--------------------------------joysick
//	Joystick js(getJoystickConfig(0));

	//--------------------------------servo
	FutabaS3010::Config servo_Config;
	servo_Config.id=0;
	FutabaS3010 servo(servo_Config);
	servo.SetDegree(SERVO_CENTRE);
	servoP = &servo;

//	--------------------------------motor
	DirMotor::Config motor_Config;
	motor_Config.id=0;
	DirMotor motorL(motor_Config);
	motorLP = &motorL;
	motor_Config.id=1;
	DirMotor motorR(motor_Config);
	motorRP = &motorR;
	motorSetPower(150, 150);

//	--------------------------------encoder
	DirEncoder::Config enc1;
	enc1.id = 0;
	DirEncoder encoderL(enc1);
	encoderLP = &encoderL;

	DirEncoder::Config enc2;
	enc2.id = 1;
	DirEncoder encoderR(enc2);
	encoderRP = &encoderR;

// //	--------------------------------compass
// 	Qmc5883::Config compassC;
// 	Qmc5883 compass(compassC);
// 	compassP = &compass;
// //	max_x: 1045,	 max_y: 1546,	 max_z: 1716	min_x: -1316,	 min_y: -1250,	 min_z: -528
// //	max_x: 1195,	 max_y: 1485,	 max_z: 2005	min_x: -1490,	 min_y: -1332,	 min_z: -796
// //	max_x: 1231,	 max_y: 1485,	 max_z: 2005	min_x: -1490,	 min_y: -1345,	 min_z: -796
// //	max_x: 1325,	 max_y: 1056,	 max_z: -63, 	min_x: -1218,	 min_y: -1410,	 min_z: -441
// //	max_x: 1251,	 max_y: 823,	 max_z: -176	min_x: -1302,	 min_y: -1605,	 min_z: -502


// //	max_x: 1156,	 max_y: 848,	 max_z: -110	min_x: -1370,	 min_y: -1586,	 min_z: -528
// //	max_x: 1188,	 max_y: 938,	 max_z: -117	min_x: -1417,	 min_y: -1597,	 min_z: -500

// //	max_x: 1168,	 max_y: 916,	 max_z: 131		min_x: -1425,	 min_y: -1443,	 min_z: -407
// //	max_x: 1196,	 max_y: 592,	 max_z: -932	min_x: -1095,	 min_y: -1556,	 min_z: -1835
// //	max_x: 1097,	 max_y: 686,	 max_z: -1046	min_x: -1380,	 min_y: -1675,	 min_z: -1897

// //	{
// //		std::array<int16_t, 3> temp1 = {-1380, -1675, -1897};
// //		compass.SetMagMin(temp1);
// //		std::array<int16_t, 3> temp2 = {1097, 686, -1046};
// //		compass.SetMagMax(temp2);
// //	}
// //
// //	//get the initial reference
// //	updateCompassAngle();
// //	updateCompassAngle();
// //	updateCompassAngle();
// //	updateCompassAngle();
// //	updateCompassAngle();
// //	float compassAngleReference = compassAngle;

	while(!startTheCarProcess)
	{
		ledP[0]->Switch();
		ledP[1]->Switch();
		ledP[2]->Switch();
		bluetoothP[0]->SendStr("g");
		System::DelayMs(20);
	}

	ledP[2]->SetEnable(false);


	/*
	if the main loop is running the led 0 will blink

	if the angle calculation is running the led 1 will blink
	*/


	uint32_t lastTime = System::Time();
	while(true)
	{
		if( ( System::Time() - lastTime ) >= 50)
		{
			lastTime = System::Time();
			ledP[0]->Switch();




			//need to disable while testing with motor set power 0
			// dodgeObstacle();








//			//calculate car beacon angle with angle in image and car drifted angle
//			//update car drifted angle from mpu
//			mpuP->Update();
//			for(int i = 0; i < 3; i++)
//			{
//				acc[i] = mpuP->GetAccel()[i];
//				gyo[i] = mpuP->GetOmega()[i];
//			}
//
//			//car heading angle from gyroscope
//			carHeadingAngle = carHeadingAngle + ( gyo[2] / countConstant);
//			if(carHeadingAngle > 180)
//			{	carHeadingAngle = carHeadingAngle - 360;	}
//			else if(carHeadingAngle < -180)
//			{	carHeadingAngle = 360 + carHeadingAngle;	}
//
//
//
//
//// 			char buffer[50];
//// 			sprintf(buffer, "carHeadingAngle: %f\n", carHeadingAngle);
//// 			bluetoothP[0]->SendStr(buffer);
//
//// //			bluetoothP->SendStr("acc  \t");
//// //			for(int i = 0; i < 3; i++)
//// //			{
//// //				char buffer[60];
//// //				sprintf(buffer, "%d: %d\t", i, acc[i]);
//// //				bluetoothP->SendStr(buffer);
//// //			}
//// //			bluetoothP->SendStr("\n");






			//angle calculation
			if ((beacon != Coor()) && carT != Coor())
			{
				ledP[1]->Switch();
				Coor carCentre((carT.x + carH.x)/2, (carT.y + carH.y)/2);
				int16_t dx = beacon.x - carCentre.x;
				int16_t dy = beacon.y - carCentre.y;

				//calculate local angle
				uint32_t hyp = distanceSquare(carCentre, beacon);
				if (hyp != 0)
				{
					if (dx == 0)
					{
						if (dy > 0)
						{
							carAngleError = 179.999999999;
						} else
						{
							carAngleError = 0.0;
						}
					} else //dx != 0
					{
						float inputX = dx / Math::Sqrt2(hyp);
						if (dy == 0)
						{
							if (dx > 0)
							{
								carAngleError = -90.0;
							} else
							{
								carAngleError = 90.0;
							}
						} else if (dx > 0)
						{
							if (dy > 0)
							{
								carAngleError = Math::ArcSin(inputX);
								carAngleError = carAngleError * 180 / PI;
								carAngleError = -180 + carAngleError;
							} else
							{
								carAngleError = Math::ArcSin(inputX);
								carAngleError = carAngleError * 180 / PI;
								carAngleError = -carAngleError;
							}
						} else
						{
							if (dy > 0)
							{
								carAngleError = Math::ArcSin(-inputX);
								carAngleError = carAngleError * 180 / PI;
								carAngleError = 180 - carAngleError;
							} else
							{
								carAngleError = Math::ArcSin(-inputX);
								carAngleError = carAngleError * 180 / PI;
							}
						}
					}

					char buffer[200];
					//for testing new bluetooth code
					lcdP->SetRegion(Lcd::Rect(0, 0, 128, 50));
					sprintf(buffer, "Bea: %d %d\nCH:\t%d %d\nCT:\t%d %d\n", beacon.x,
						beacon.y, carH.x, carH.y, carT.x, carT.y);
					writerP->WriteString(buffer);

					lcdP->SetRegion(Lcd::Rect(0, 60, 128, 50));
					sprintf(buffer, "Error: %f\nAAngle: %f\n", carAngleError, angleTracking(carT, carH));
					writerP->WriteString(buffer);


//						char buffer[100];
//						sprintf(buffer, "%f\n", carAngleError);
//						bluetoothP->SendStr(buffer);
					carAngleError -= angleTracking(carT, carH);
					boundAngle(carAngleError);
// 						char buffer[100];
// 						sprintf(buffer, "hyp: %d\tHeadingAngle: %f\tAngleError: %f\n", hyp, carHeadingAngle, carAngleError);
// 						bluetoothP[1]->SendStr(buffer);

					//may do something with the distance and speed
				}
			}

			//must have for steering
			if (moveBack == false && moveForward == false)
			{
				//control servo and motor
				servoOutput = SERVO_CENTRE + (int16_t)(carAngleError * servoKP
					+ (carAngleError - carAngleErrorPrevious) * servoKD);

				if (servoOutput > SERVO_LEFT_LIMIT)
					servoOutput = SERVO_LEFT_LIMIT;
				else if (servoOutput < SERVO_RIGHT_LIMIT)
					servoOutput = SERVO_RIGHT_LIMIT;

				carAngleErrorPrevious = carAngleError;
				servo.SetDegree(servoOutput);
			}

		}//end if for checking time
	}//end while loop

	return 0;
}

// float carLocate()
// {
// 	int coorSize = Car.size();
// 	int cDistSQ = DISTANCE_CHECK * DISTANCE_CHECK;

// 	if (coorSize > 1)
// 	{
// 		if (distanceSquare(Car[0], Car[coorSize - 1]) < cDistSQ)
// 		{
// 			//	Check current Beacon Coor and Latest Coor
// 			Car[0] = Car[coorSize - 1];

// 			//	If equal, update current Beacon Coor
// 			for ( int i = 0; i < (coorSize -1); i++)
// 			{	Car.pop_back();	}
// 		}
// 		if (Car.size() == 5)
// 		{
// 			//	If latest and current Car Coor different for 4 times
// 			//	Check the middle three records
// 			//	If similar, take the third Coor
// 			//	Else, use the latest Coor directly
// 			if ((distanceSquare(Car[1], Car[2]) < cDistSQ) && (distanceSquare(Car[1], Car[3]) < cDistSQ)
// 					&& (distanceSquare(Car[2], Car[3]) < cDistSQ))
// 			{	Car[0] = Car[3];	}
// 			else { Car[0] = Car[4]; }
// 			for ( int i = 0; i < 4; i++)
// 			{	Car.pop_back();	}
// 		}
// 	}
// 	// Return squared distance
// 	return distanceSquare(Car[0] ,Beacon);
// }

uint32_t distanceSquare(const Coor& a, const Coor& b)
{
	return (a.x -b.x) *(a.x -b.x) + (a.y -b.y) *(a.y -b.y);
}		// (x2-x1)^2 + (y2-y1)^2

void boundAngle(float& angle)
{
	if (angle < -180)
	{
		angle += 360;
	}else if(angle > 180)
	{
		angle -= 360;
	}
}

void motorSetPower(int16_t pL, int16_t pR)
{
	//go forward
	if(pL >= 0)
	{
		motorLP->SetClockwise(true); // for left motor, true == forward
		motorLP->SetPower(pL);
	}else
	{
		motorLP->SetClockwise(false);
		motorLP->SetPower(-pL);
	}

	//go forward
	if(pR >= 0)
	{
		motorRP->SetClockwise(false); // for right motor, false == forward
		motorRP->SetPower(pR);
	}else
	{
		motorRP->SetClockwise(true);
		motorRP->SetPower(-pR);
	}
}

// float updateCompassAngle()
// {
// 	std::array<int16_t, 3> magRaw;
// 	std::array<int16_t, 3> magMax;
// 	std::array<int16_t, 3> magMin;
// 	std::array<int32_t, 3> magNormalized;
// //	compassP->Update();
// 	compassP->UpdateWithCalibration();
// 	magRaw = compassP->GetMag();
// 	magMax = compassP->GetMagMax();
// 	magMin = compassP->GetMagMin();
// 	magNormalized = compassP->GetNormalizedMag();

// 	 char bluetoothBuffer[150];
// 	 sprintf(bluetoothBuffer, "raw_x: %d,\t raw_y: %d,\t raw_z: %d\t", magRaw[0], magRaw[1], magRaw[2]);
// 	 bluetoothP[0]->SendStr(bluetoothBuffer);

// 	 sprintf(bluetoothBuffer, "max_x: %d,\t max_y: %d,\t max_z: %d\t", magMax[0], magMax[1], magMax[2]);
// 	 bluetoothP[0]->SendStr(bluetoothBuffer);

// 	 sprintf(bluetoothBuffer, "min_x: %d,\t min_y: %d,\t min_z: %d\t", magMin[0], magMin[1], magMin[2]);
// 	 bluetoothP[0]->SendStr(bluetoothBuffer);

// 	sprintf(bluetoothBuffer, "norm_x: %d,\t norm_y: %d,\t norm_z: %d\t", magNormalized[0], magNormalized[1], magNormalized[2]);
// 	bluetoothP[0]->SendStr(bluetoothBuffer);

// 	//calculate the acute angle first
// 	compassAngle = 0.0;
// 	if (magNormalized[1] > 0)
// 	{
// 		if (magNormalized[1] < 10000)
// 		{
// 			compassAngle = Math::ArcSin(magNormalized[1] / 10000.0);
// 		}else
// 		{
// 			compassAngle = PI / 2;
// 		}
// 	}else if (magNormalized[1] < 0)
// 	{
// 		if (magNormalized[1] > -10000)
// 		{
// 			compassAngle = Math::ArcSin( -magNormalized[1] / 10000.0);
// 		}else
// 		{
// 			compassAngle = PI / 2;
// 		}
// 	}
// 	//do nothing if magNormalized[1] == 0

// 	//in degree
// 	compassAngle = compassAngle * 180 / PI;

// 	if (magNormalized[1] > 0)
// 	{
// 		if (magNormalized[0] >= 0)
// 		{
// 			//do nothing
// 		}else
// 		{
// 			compassAngle = 180 - compassAngle;
// 		}
// 	}else
// 	{
// 		if (magNormalized[0] >= 0)
// 		{
// 			compassAngle = 360 - compassAngle;
// 		}else
// 		{
// 			compassAngle = 180 + compassAngle;
// 		}
// 	}

// 	sprintf(bluetoothBuffer, "compass angle: %f\n", compassAngle);
// 	bluetoothP[0]->SendStr(bluetoothBuffer);
// }

void dodgeObstacle()
{
	//update encoder
	encoderLP->Update();
	encoderRP->Update();
	eReadingL = encoderLP->GetCount();
	eReadingR = (-encoderRP->GetCount());

	// //send encoder count for debugging
	// char bluetoothBuffer[100];
	// sprintf(bluetoothBuffer, "encoderL: %d\t encoderR: %d\n", eReadingL, eReadingR);
	// bluetoothP[0]->SendStr(bluetoothBuffer);

	//blocked by something so the count become very small
	if ((eReadingL < 20) && (eReadingR < 20) && (eReadingR >= 0) && (eReadingR >= 0))
	{
		if(moveForward == false)
			moveBack = true;
	}

	//move backward
	if (moveBack == true)
	{
		//for some distance
		if ((accEncR < (COUNT_FOR_ONE_METER / 18)) && (accEncL < (COUNT_FOR_ONE_METER / 18)))
		{
			accEncR += encoderRP->GetCount();
			accEncL -= encoderLP->GetCount();
			motorSetPower(-220, -220);

			//turn more to the same direction
			if (servoP->GetDegree() > SERVO_CENTRE)
			{
				servoP->SetDegree(SERVO_LEFT_LIMIT);
			}
			else
			{
				servoP->SetDegree(SERVO_RIGHT_LIMIT);
			}
		}else //move forward again by setting the flag
		{
			accEncR = 0;
			accEncL = 0;
			motorSetPower(170, 170);
			moveBack = false;
			moveForward = true;
		}
	}else if (moveForward == true)
	{
		if ((accEncR < (COUNT_FOR_ONE_METER / 48)) && (accEncL < (COUNT_FOR_ONE_METER / 48)))
		{
			accEncR -= encoderRP->GetCount();
			accEncL += encoderLP->GetCount();
			servoP->SetDegree(SERVO_CENTRE);
		}else
		{
			accEncR = 0;
			accEncL = 0;
			moveForward = false;
		}
	}
}

float angleTracking(Coor carT, Coor carH)
{
	//it is assumed that carT and carH is not the same point
	// -180 to 180 deg

	int deltaX = carH.x - carT.x;
	int deltaY = carH.y - carT.y;

	if (deltaX != 0)
	{
		// The degree that the image has rotated
		// rad -> deg
		float angle = 180 * Math::ArcTan((float) abs(deltaY) / abs(deltaX)) / 3.141592654;
		if (deltaX > 0)
		{
			if (deltaY > 0)
			{
				return (-90 - angle);
			} else
			{
				return (-90 + angle);
			}
		} else
		{
			if (deltaY > 0)
			{
				return (90 + angle);
			} else
			{
				return (90 - angle);
			}
		}
	} else
	{
		if (deltaY > 0)
		{
			return 179.999999999;
		} else
		{	
			return 0;
		}
	}
}

St7735r::Config getLcdConfig()
{
//	/// Revert the screen upside down
//	bool is_revert = false;
//	/// Whether using a BGR panel instead of a RGB one
//	bool is_bgr = false;
//	/// Frame rate of the screen
//	uint8_t fps = 60;
	return St7735r::Config();
}

Joystick::Config getJoystickConfig(uint8_t _id)
{
	Joystick::Config c;
	c.id = _id;
	c.is_active_low = 1;
	for(int i=0; i<5; i++)
		c.listener_triggers[i] = c.Trigger::kDown;
	c.handlers[0] = [](const uint8_t id, const Joystick::State which)
		{
		};
	c.handlers[1] = [](const uint8_t id, const Joystick::State which)
		{
		};
	c.handlers[2] = [](const uint8_t id, const Joystick::State which)
		{
		};
	c.handlers[3] = [](const uint8_t id, const Joystick::State which)
		{
		};
	c.handlers[4] = [](const uint8_t id, const Joystick::State which)
		{

		};

	return c;
}

Mpu6050::Config getMpuConfig()
{
	Mpu6050::Config c;
	c.gyro_range = Mpu6050::Config::Range::kLarge;
	c.accel_range = Mpu6050::Config::Range::kMid;
	c.cal_drift = true;
	return c;
}

bool bluetoothListenerOne(const Byte* data, const size_t size)
{
	ledP[3]->Switch();

	if(startFlag == false){
		if (*data == 's')
		{
			startFlag = true;
		}
	}else
	{
		if(*data <= '9' && *data >= '0')
		{
			messageFromDrone += *data;
		}else if(*data == ',')
		{
			int32_t tempCoordinate = std::stoi(messageFromDrone);
			coorBuffer.push_back(tempCoordinate);
			messageFromDrone.clear();
		}else if(*data == 'e')
		{
			beacon = Coor(coorBuffer[0], coorBuffer[1]);
			carH = Coor(coorBuffer[2], coorBuffer[3]);
			carT = Coor(coorBuffer[4], coorBuffer[5]);
			coorBuffer.clear();
			startFlag = false;
		}
	}

	if(*data == 'g')
	{
		startTheCarProcess = true;
	}

	return true;
}

bool bluetoothListenerTwo(const Byte* data, const size_t size)
{
	return true;
}

