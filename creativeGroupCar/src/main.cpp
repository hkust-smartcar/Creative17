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

#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include <libsc/led.h>
#include <libsc/button.h>
#include <libbase/k60/gpio.h>
#include <libbase/k60/pin.h>

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
    {	return vec(x+v.x, y+v.y, z+v.z);	}

    vec operator-(const vec& v)
    {	return vec(x-v.x, y-v.y, z-v.z);	}

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
uint8_t MEAN_FILTER_WINDOW_SIZE = 3;	//window size should be odd
//#define ENABLE_LCD
#define DISTANCE_CHECK 20
#define DATA_WIDTH 4
const float countConstant = 100000;
#define PI 3.14159265358979323846

#define SERVO_LEFT_LIMIT 1140
#define SERVO_CENTRE 850
#define SERVO_RIGHT_LIMIT 565

#define COUNT_FOR_ONE_METER 57400

#define DISTANCE_FOR_UNLOCK_SERVO 1700



//the centre received through bluetooth will be stored in the global variable centre
//so use centre to do process also
float carLocate();
int32_t distanceSquare(const Coor& a, const Coor& b);
void motorSetPower(int16_t pL, int16_t pR);


//k60::Ov7725::Config getCameraConfig();

UartDevice::Config getBluetoothConfig();
Mpu6050::Config getMpuConfig();
St7735r::Config getLcdConfig();
Joystick::Config getJoystickConfig(uint8_t _id);

//assume receive centre data in the following format
//sXXX,XXX,XXX,XXX,e the first Coor is beacon
bool bluetoothListenerOne(const Byte* data, const size_t size);
bool bluetoothListenerTwo(const Byte* data, const size_t size);
//void B1Listener(const uint8_t);
//void B2Listener(const uint8_t);

Led* ledP[4];
St7735r* lcdP;

JyMcuBt106* bluetoothP[2];
Joystick* joystickP;
LcdTypewriter* writerP;
Mpu6050* mpuP;
DirMotor* motorLP;
DirMotor* motorRP;


std::vector<Coor> Car;
Coor Beacon;
bool startFlag = false;
float distance = 0.0f;

std::array<int32_t, 3> acc = {0, 0, 0};
std::array<int32_t, 3> gyo = {0, 0, 0};
std::vector<vec> accSample;
float carHeadingAngle;
float carAngleError; //positive need to turn left, negative need to turn right
std::string messageFromDrone;
std::vector<int16_t> coorBuffer;

bool startTheCarProcess = false;

float servoKp = 20;
//PID servoControl;
uint16_t servoOutput;
int32_t distanceForLockServo = 0;
uint8_t countForLockServoL = 0;
uint8_t countForLockServoR = 0;

float motorLKp = 1;
float motorRKp = 1;
//PID motorLControl;
//PID motorRControl;
int16_t Lspeed = 0;
int16_t Rspeed = 0;
int32_t eReadingL = 0;
int32_t eReadingR = 0;

PID motorLControl(motorLKp,0,0);
PID motorRControl(motorRKp,0,0);
PID servoControl(servoKp, 0, 0);

bool programEnd = false;

uint16_t servoAngleSpecial = SERVO_CENTRE + 80;

int main(void)
{
	System::Init();

	//	--------------------------------motor
		DirMotor::Config motor_Config;
		motor_Config.id=0;
		DirMotor motorL(motor_Config);
		motorLP = &motorL;
		motor_Config.id=1;
		DirMotor motorR(motor_Config);
		motorRP = &motorR;
		Lspeed = 180;
		Rspeed = 180;
		motorSetPower(Lspeed, Rspeed);

	//---------------------------------button
	Button::Config cB1;
	cB1.id = 0;
	cB1.is_active_low = true;
	cB1.is_use_pull_resistor = true;
	//cB1.listener = &B1Listener;
	cB1.listener =  [](const uint8_t id)
						{
							servoAngleSpecial -= 30;
						};
	cB1.listener_trigger = Button::Config::Trigger::kDown;
	Button button1(cB1);

	Button::Config cB2;
	cB2.id = 1;
	cB2.is_active_low = true;
	cB2.is_use_pull_resistor = true;
	//cB2.listener = &B2Listener;
	cB2.listener =  [](const uint8_t id)
						{
							servoAngleSpecial += 30;
						};
	cB2.listener_trigger = Button::Config::Trigger::kDown;
	Button button2(cB2);



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
//	c.rx_irq_threshold = 1;
//	c.tx_buf_size = ;
	bluetoothC.rx_isr = bluetoothListenerOne;
	JyMcuBt106 bt1(bluetoothC);
	bluetoothP[0] = &bt1;

//	bluetoothC.id = 1;
//	bluetoothC.rx_isr = bluetoothListenerTwo;
//	JyMcuBt106 bt2(bluetoothC);
//	bluetoothP[1] = &bt2;


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

//	--------------------------------encoder
	DirEncoder::Config enc1;
	enc1.id = 0;
	DirEncoder encoderL(enc1);

	DirEncoder::Config enc2;
	enc2.id = 1;
	DirEncoder encoderR(enc2);


	bool irOn = false;
//	int servoDegreeTemp = 0;
	bool lockServo = false;
	bool moveBack = false;
	bool moveForward = false;

	char rangeBuffer[50];
	int32_t accEncL = 0;
	int32_t accEncR = 0;

//	while(!startTheCarProcess)
//	{
//		ledP[0]->Switch();
//		ledP[1]->Switch();
//		ledP[2]->Switch();
//		bluetoothP[0]->SendStr("g");
//		System::DelayMs(20);
//	}

	// When Car is ready
	ledP[2]->SetEnable(true);
//	ledP[2]->SetEnable(false);
	System::DelayMs(1500);

	servo.SetDegree(servoAngleSpecial);

	uint32_t lastTime = System::Time();
	while(true)
	{

		if( ( System::Time() - lastTime ) >= 50)
		{
			lastTime = System::Time();
			ledP[0]->Switch();

			//switching ir led
			if(irOn)
			{
				ir.Reset();
				irOn = false;
			}else
			{
				ir.Set();
				irOn = true;
			}
		//---------------------------------------
			servo.SetDegree(servoAngleSpecial);

			//for dodging beacon
			encoderL.Update();
			encoderR.Update();
			eReadingL = encoderL.GetCount();
			eReadingR = (-encoderR.GetCount());
			char bluetoothBuffer[100];
			sprintf(bluetoothBuffer, "encoderL: %d\t encoderR: %d\n", eReadingL, eReadingR);
			bluetoothP[0]->SendStr(bluetoothBuffer);

			//blocked by something
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
					accEncR += encoderR.GetCount();
					accEncL -= encoderL.GetCount();
					motorSetPower(-220, -220);

					//turn more to the same direction
					if (servo.GetDegree() > SERVO_CENTRE)
					{	servo.SetDegree(SERVO_LEFT_LIMIT);	}
					else
					{	servo.SetDegree(SERVO_RIGHT_LIMIT);	}
				}else //move forward again by setting the flag
				{
					accEncR = 0;
					accEncL = 0;
					Lspeed = 170;
					Rspeed = 170;
					motorSetPower(Lspeed, Rspeed);
					moveBack = false;
					moveForward = true;
				}
			}else if (moveForward == true)
			{
				if ((accEncR < (COUNT_FOR_ONE_METER / 48)) && (accEncL < (COUNT_FOR_ONE_METER / 48)))
				{
					accEncR -= encoderR.GetCount();
					accEncL += encoderL.GetCount();
					servoAngleSpecial = SERVO_CENTRE;
				}else
				{
					accEncR = 0;
					accEncL = 0;
					moveForward = false;
				}
			}
			else
			{
				Lspeed = motorLControl.output(220, Lspeed);
				Rspeed = motorRControl.output(220, Rspeed);
				motorSetPower(Lspeed, Rspeed);
			}
//
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
////			char buffer[50];
////			sprintf(buffer, "%f\n", carHeadingAngle);
////			bluetoothP->SendStr(buffer);
//
////			bluetoothP->SendStr("acc  \t");
////			for(int i = 0; i < 3; i++)
////			{
////				char buffer[60];
////				sprintf(buffer, "%d: %d\t", i, acc[i]);
////				bluetoothP->SendStr(buffer);
////			}
////			bluetoothP->SendStr("\n");
//
//			//add them up
//			if((Beacon != Coor()) && (Car.size() > 0))
//			{
//				if(Car[0] != Coor())
//				{
//					ledP[1]->Switch();
//					int16_t dx = Car[0].x - Beacon.x;
//					int16_t dy = Car[0].y - Beacon.y;
//					//calculate local angle
//					uint32_t hyp = distanceSquare(Car[0], Beacon);
//
//					if (hyp != 0)
//					{
//						if(dx == 0)
//						{
//							dy > 0? carAngleError = 0.0f : carAngleError = 179.99999999999f;
//						}else
//						{
//							float inputX = dx / Math::Sqrt2(hyp);
//							if(dy == 0)
//							{
//								dx > 0? carAngleError = 90.0f : carAngleError = -90.0f;
//							}else if(dx > 0)
//							{
//								if(dy > 0)
//								{
//									carAngleError = Math::ArcSin(inputX);
//									carAngleError = carAngleError * 180 / PI;
//								}else
//								{
//									carAngleError = Math::ArcSin(inputX);
//									carAngleError = carAngleError * 180 / PI;
//									carAngleError = 180 -  carAngleError;
//								}
//							}else
//							{
//								if(dy > 0)
//								{
//									carAngleError = -Math::ArcSin(-inputX);
//									carAngleError = carAngleError * 180 / PI;
//								}else
//								{
//									carAngleError = -Math::ArcSin(-inputX);
//									carAngleError = carAngleError * 180 / PI;
//									carAngleError = -180 - carAngleError;
//								}
//
//							}
//						}
////						char buffer[100];
////						sprintf(buffer, "%f\n", carAngleError);
////						bluetoothP->SendStr(buffer);
//						carAngleError -= carHeadingAngle;
//						if(carAngleError > 180)
//						{
//							carAngleError = carAngleError - 360;
//						}else if(carAngleError < -180)
//						{
//							carAngleError = 360 + carHeadingAngle;
//						}
////						char buffer[100];
////						sprintf(buffer, "hyp: %d\tHeadingAngle: %f\tAngleError: %f\n", hyp, carHeadingAngle, carAngleError);
////						bluetoothP[1]->SendStr(buffer);
//					}else
//					{
//						//distance == 0
//						//determine what to do later
//					}
//				}
//			}else
//			{
//				//no information of how to move
//				motorSetPower(0, 0);
//			}
//
//			if (moveBack == false && moveForward == false)
//			{
//				//control servo and motor
//				servoOutput = SERVO_CENTRE + (int16_t)(carAngleError * servoKp);
//			//	servoOutput = servoControl.output((SERVO_CENTRE + carAngleError), servo.GetDegree());

//				if(servoOutput > SERVO_LEFT_LIMIT)
//					servoOutput = SERVO_LEFT_LIMIT;
//				else if(servoOutput < SERVO_RIGHT_LIMIT)
//					servoOutput = SERVO_RIGHT_LIMIT;
//				servo.SetDegree(servoOutput);
//			}

		}//end if for checking time
	}//end while loop

	return 0;
}

float carLocate()
{
	int coorSize = Car.size();
	int cDistSQ = DISTANCE_CHECK * DISTANCE_CHECK;

	if (coorSize > 1)
	{
		if (distanceSquare(Car[0], Car[coorSize - 1]) < cDistSQ)
		{
			//	Check current Beacon Coor and Latest Coor
			Car[0] = Car[coorSize - 1];

			//	If equal, update current Beacon Coor
			for ( int i = 0; i < (coorSize -1); i++)
			{	Car.pop_back();	}
		}
		if (Car.size() == 5)
		{
			//	If latest and current Car Coor different for 4 times
			//	Check the middle three records
			//	If similar, take the third Coor
			//	Else, use the latest Coor directly
			if ((distanceSquare(Car[1], Car[2]) < cDistSQ) && (distanceSquare(Car[1], Car[3]) < cDistSQ)
					&& (distanceSquare(Car[2], Car[3]) < cDistSQ))
			{	Car[0] = Car[3];	}
			else { Car[0] = Car[4]; }
			for ( int i = 0; i < 4; i++)
			{	Car.pop_back();	}
		}
	}
	// Return squared distance
	return distanceSquare(Car[0] ,Beacon);
}

int32_t distanceSquare(const Coor& a, const Coor& b)
{	return (a.x -b.x) *(a.x -b.x) + (a.y -b.y) *(a.y -b.y);	}		// (x2-x1)^2 + (y2-y1)^2

void motorSetPower(int16_t pL, int16_t pR)
{
	//go forward
	if(pL >= 0)
	{
		motorLP->SetClockwise(true); // for left motor, true == forward
		motorLP->SetPower(150);
	}else
	{
		motorLP->SetClockwise(false);
		motorLP->SetPower(150);
	}

	//go forward
	if(pR >= 0)
	{
		motorRP->SetClockwise(false); // for right motor, false == forward
		motorRP->SetPower(150);
	}else
	{
		motorRP->SetClockwise(true);
		motorRP->SetPower(150);
	}
}

/*
k60::Ov7725::Config getCameraConfig()
{
	k60::Ov7725::Config c;
	c.id = 0;
	c.w = CAM_W;
	c.h = CAM_H;
	c.fps = k60::Ov7725Configurator::Config::Fps::kHigh;
	return c;
}
*/
//UartDevice::Config getBluetoothConfig()
//{
//	UartDevice::Config c;
//	c.id = 1;
//	c.baud_rate = Uart::Config::BaudRate::k115200;
////	c.rx_irq_threshold = 1;
////	c.tx_buf_size = ;
//	c.rx_isr = bluetoothListener;
//	return c;
//}

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
			MEAN_FILTER_WINDOW_SIZE++;
		};
	c.handlers[1] = [](const uint8_t id, const Joystick::State which)
		{
			MEAN_FILTER_WINDOW_SIZE--;
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
	if( (*data <= '9' && *data >= '0') || *data == ',' || *data == 's' || *data == 'e')
	{
		if(*data == 's')
		{
	//		std::string temp;
	//		temp += *data;
	//		temp += '\n';
	//		bluetoothP->SendStr(temp);
			startFlag = true;
		}else if(*data == 'e')
		{
			Beacon = Coor(coorBuffer[0], coorBuffer[1]);
			if(Car.size() == 0)
				Car.push_back(Coor(coorBuffer[2], coorBuffer[3]));
			else
				Car.front() = Coor(coorBuffer[2], coorBuffer[3]);
			coorBuffer.clear();
			startFlag = false;

			//update the beacon coordinate by history
//			carLocate();
		}else if(*data == ',')
		{
			int32_t tempCoordinate = std::stoi(messageFromDrone);
			coorBuffer.push_back(tempCoordinate);
			messageFromDrone.clear();
		}else if(startFlag == true)
		{
			messageFromDrone += *data;
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
