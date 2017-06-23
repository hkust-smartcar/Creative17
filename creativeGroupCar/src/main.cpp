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
#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include <libsc/led.h>
#include <libsc/button.h>
#include <libbase/k60/gpio.h>
#include <libbase/k60/pin.h>
#include <libsc/k60/ov7725.h>
#include <libsc/k60/ov7725_configurator.h>
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
uint8_t MEAN_FILTER_WINDOW_SIZE = 3;	//window size should be odd
//#define ENABLE_LCD
#define DISTANCE_CHECK 20
#define DATA_WIDTH 4
const float countConstant = 100000;
#define PI 3.14159265358979323846

#define SERVO_LEFT_LIMIT 1100
#define SERVO_CENTRE 824
#define SERVO_RIGHT_LIMIT 550



//the centre received through bluetooth will be stored in the global variable centre
//so use centre to do process also
float carLocate();
uint32_t distanceSquare(const Coor& a, const Coor& b);


k60::Ov7725::Config getCameraConfig();
UartDevice::Config getBluetoothConfig();
Mpu6050::Config getMpuConfig();
St7735r::Config getLcdConfig();
Joystick::Config getJoystickConfig(uint8_t _id);

//assume receive centre data in the following format
//startXXX,XXX,XXX,XXX,end!
bool bluetoothListener(const Byte* data, const size_t size);



Led* ledP[4];
St7735r* lcdP;
Ov7725* cameraP;
JyMcuBt106* bluetoothP;
Joystick* joystickP;
LcdTypewriter* writerP;
Mpu6050* mpuP;
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
float servoKp = 15;
uint16_t servoOutput;


int main(void)
{
	System::Init();

//	//---------------------------------button
//	Button::Config buttonC;
//	buttonC.is_use_pull_resistor = false;
//	buttonC.is_active_low = true;
//	buttonC.id = 0;
//	buttonC.listener_trigger = Button::Config::Trigger::kDown;
//	buttonC.listener =  [](const uint8_t id)
//						{
//							for(int i = 0; i < 3; i++)
//							{
//								char buffer[50];
//								sprintf(buffer, "%d: %d\t", i, acc[i]);
//								bluetoothP->SendStr(buffer);
//							}
//						};

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
	JyMcuBt106 bt(getBluetoothConfig());
	bluetoothP = &bt;

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

//	--------------------------------motor
	DirMotor::Config motor_Config;
	motor_Config.id=0;
	DirMotor motorL(motor_Config);
	motor_Config.id=1;
	DirMotor motorR(motor_Config);
	motorL.SetClockwise(true); // for left motor, true == forward
	motorL.SetPower(200);
	motorR.SetClockwise(false); // for right motor, false == forward
	motorR.SetPower(200);

	bool irOn = false;

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

//			//calculate car beacon distance
//			distance = distanceSquare(Beacon, Car[0]);

			//calculate car beacon angle with angle in image and car drifted angle
			//update car drifted angle from mpu
			mpuP->Update();
			for(int i = 0; i < 3; i++)
			{
				acc[i] = mpuP->GetAccel()[i];
				gyo[i] = mpuP->GetOmega()[i];
			}

			//car heading angle from gyroscope
			carHeadingAngle = carHeadingAngle + ( gyo[2] / countConstant);
			if(carHeadingAngle > 180)
			{
				carHeadingAngle = carHeadingAngle - 360;
			}else if(carHeadingAngle < -180)
			{
				carHeadingAngle = 360 + carHeadingAngle;
			}

//			char buffer[50];
//			sprintf(buffer, "%f\n", carHeadingAngle);
//			bluetoothP->SendStr(buffer);

//			bluetoothP->SendStr("acc  \t");
//			for(int i = 0; i < 3; i++)
//			{
//				char buffer[60];
//				sprintf(buffer, "%d: %d\t", i, acc[i]);
//				bluetoothP->SendStr(buffer);
//			}
//			bluetoothP->SendStr("\n");

			//add them up
			if((Beacon != Coor()) && (Car.size() > 0))
			{
				if(Car[0] != Coor())
				{
					ledP[1]->Switch();
					int16_t dx = Car[0].x - Beacon.x;
					int16_t dy = Car[0].y - Beacon.y;
					//calculate local angle
					uint32_t hyp = distanceSquare(Car[0], Beacon);
					if(hyp != 0)
					{
						if(dx == 0)
						{
							dy > 0? carAngleError = 0.0f : carAngleError = 179.99999999999f;
						}else
						{
							float inputX = dx / Math::Sqrt2(hyp);
							if(dy == 0)
							{
								dx > 0? carAngleError = 90.0f : carAngleError = -90.0f;
							}else if(dx > 0)
							{
								if(dy > 0)
								{
									carAngleError = Math::ArcSin(inputX);
									carAngleError = carAngleError * 180 / PI;
								}else
								{
									carAngleError = Math::ArcSin(inputX);
									carAngleError = carAngleError * 180 / PI;
									carAngleError = 180 -  carAngleError;
								}
							}else
							{
								if(dy > 0)
								{
									carAngleError = -Math::ArcSin(-inputX);
									carAngleError = carAngleError * 180 / PI;
								}else
								{
									carAngleError = -Math::ArcSin(-inputX);
									carAngleError = carAngleError * 180 / PI;
									carAngleError = -180 - carAngleError;
								}

							}
						}
//						char buffer[100];
//						sprintf(buffer, "%f\n", carAngleError);
//						bluetoothP->SendStr(buffer);
						carAngleError -= carHeadingAngle;
						if(carAngleError > 180)
						{
							carAngleError = carAngleError - 360;
						}else if(carAngleError < -180)
						{
							carAngleError = 360 + carHeadingAngle;
						}
//						char buffer[100];
//						sprintf(buffer, "hyp: %d\tHeadingAngle: %f\tAngleError: %f\n", hyp, carHeadingAngle, carAngleError);
//						bluetoothP->SendStr(buffer);
					}else
					{
						//distance == 0
						//determine what to do later
					}
				}
			}

			//control servo and motor
			servoOutput = SERVO_CENTRE + (int16_t)(carAngleError * servoKp);
			if(servoOutput > SERVO_LEFT_LIMIT)
				servoOutput = SERVO_LEFT_LIMIT;
			else if(servoOutput < SERVO_RIGHT_LIMIT)
				servoOutput = SERVO_RIGHT_LIMIT;
			servo.SetDegree(servoOutput);

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

uint32_t distanceSquare(const Coor& a, const Coor& b)
{	return (a.x -b.x) *(a.x -b.x) + (a.y -b.y) *(a.y -b.y);	}		// (x2-x1)^2 + (y2-y1)^2


k60::Ov7725::Config getCameraConfig()
{
	k60::Ov7725::Config c;
	c.id = 0;
	c.w = CAM_W;
	c.h = CAM_H;
	c.fps = k60::Ov7725Configurator::Config::Fps::kHigh;
	return c;
}

UartDevice::Config getBluetoothConfig()
{
	UartDevice::Config c;
	c.id = 0;
	c.baud_rate = Uart::Config::BaudRate::k115200;
//	c.rx_irq_threshold = 1;
//	c.tx_buf_size = ;
	c.rx_isr = bluetoothListener;
	return c;
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

bool bluetoothListener(const Byte* data, const size_t size)
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

	return true;
}
