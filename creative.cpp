/*
 * creative.cpp
 *
 *  Created on: Jul 22, 2017
 *      Author: Mk
 */


#include <cassert>
#include <libbase/k60/mcg.h>
#include <libsc/system.h>

#include <cstring>
#include <cmath>
#include <vector>
#include <list>
#include <array>
#include <sstream>
#include <cstring>
#include <stdio.h>

#include <camera.h>

//LED
#include <libsc/led.h>

//Button
//#include <libsc/button.h>

//LCD
//#include <libsc/st7735r.h>

//Camera
#include <libsc/k60/ov7725.h>

//Motor
#include <libsc/alternate_motor.h>

//Encoder
#include <libsc/dir_encoder.h>

//Servo
#include <libsc/futaba_s3010.h>
#include <libsc/tower_pro_mg995.h>

//Bluetooth
#include <libsc/k60/jy_mcu_bt_106.h>

//Accel and Gyro
#include <libsc/mpu6050.h>

#define X_DIR_SERVO_MIN 260
#define X_DIR_SERVO_MID 1030
#define X_DIR_SERVO_MAX 1800

#define Y_DIR_SERVO_MIN 240
#define Y_DIR_SERVO_MID 995
#define Y_DIR_SERVO_MAX 1750

namespace libbase
{
	namespace k60
	{

		Mcg::Config Mcg::GetMcgConfig()
		{
			Mcg::Config config;
			config.external_oscillator_khz = 50000;
			config.core_clock_khz = 150000;
			return config;
		}

	}
}

using namespace std;
using namespace libsc;
using namespace libbase::k60;
using namespace libsc::k60;

int main(void) {

 	System::Init();

//Loop
	uint32_t startTime = System::Time();
	long prevSampleTime = System::Time();
	double dt = 0;
	long currentTime = 0;
	long loopCounter = 0;

//Angle
	double acc[3], gyro[3];
	double accbine = 0;
	double accAngX = 0, accAngY = 0;
	double anvX = 0, anvY = 0;
	double gyroAngX = 0, gyroAngY = 0;
	double angleX = 90, angleY = 90;
	double servoAngX = 0, servoAngY = 0;
	bool state = 0;


//	Led::Config LedConfig;
//	LedConfig.id = 0;
//	Led led1(LedConfig);

//	St7735r::Config LcdConfig;
//	LcdConfig.is_revert = true;
//	St7735r Lcd1(LcdConfig);

	TowerProMg995::Config ServoConfig;
	ServoConfig.id = 0;
	TowerProMg995 servoX(ServoConfig);
	ServoConfig.id = 1;
	TowerProMg995 servoY(ServoConfig);

//	Ov7725::Config CamConfig;
//	CamConfig.id = 0;
//	CamConfig.w = width;
//	CamConfig.h = height;
//	CamConfig.fps = Ov7725Configurator::Config::Fps(2);
//	Ov7725 Cam1(CamConfig);

//	JyMcuBt106::Config BluetoothConfig;
//	BluetoothConfig.id = 0;
//	BluetoothConfig.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
//	JyMcuBt106 bluetooth1(BluetoothConfig);

	Mpu6050::Config mpuConfig;
	mpuConfig.gyro_range=Mpu6050::Config::Range::kSmall;
	mpuConfig.accel_range=Mpu6050::Config::Range::kSmall;
	Mpu6050 mpu(mpuConfig);

//	AlternateMotor::Config LeftMotorConfig;
//	LeftMotorConfig.id = 1;
//	AlternateMotor motorLeft(LeftMotorConfig);
//	leftMotorPtr = &motorLeft;
//	AlternateMotor::Config RightMotorConfig;
//	RightMotorConfig.id = 0;
//	AlternateMotor motorRight(RightMotorConfig);
//	rightMotorPtr = &motorRight;
//
//	DirEncoder::Config LeftEncoderConfig;
//	LeftEncoderConfig.id = 0;
//	DirEncoder encoderLeft(LeftEncoderConfig);
//	encoderLeftPtr = &encoderLeft;
//	DirEncoder::Config RightEncoderConfig;
//	RightEncoderConfig.id = 1;
//	DirEncoder encoderRight(RightEncoderConfig);
//	encoderRightPtr = &encoderRight;
//
//	motorLeft.SetClockwise(leftForward);
//	motorRight.SetClockwise(rightForward);
//	motorLeft.SetPower(0);
//	motorRight.SetPower(0);


	double temp = 0, temp1 = 0;
	servoX.SetDegree(X_DIR_SERVO_MID);
	servoY.SetDegree(Y_DIR_SERVO_MID);
	System::DelayMs(2000);

	startTime = System::Time();

	while(1) {

		currentTime = System::Time();

		if (currentTime-startTime >= 10) {

			startTime = currentTime;
			loopCounter++;

			//time interval
			dt = (double)(System::Time()-prevSampleTime)/1000;
			prevSampleTime = System::Time();

			//angle
			mpu.Update(1);
			acc[0] = mpu.GetAccel()[0]-1177;
			acc[1] = mpu.GetAccel()[1]-1199;
			acc[2] = mpu.GetAccel()[2]+4999;
			gyro[0] = mpu.GetOmega()[0]+69;
			gyro[1] = mpu.GetOmega()[1]+18;
			gyro[2] = mpu.GetOmega()[2]+28;
			accbine = sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
			accAngX = (acos(acc[0]/accbine)*180)/3.1415;
			accAngY = (acos(acc[1]/accbine)*180)/3.1415;
			anvX = gyro[0]/160/131;
			anvY = gyro[1]/160/131;
			gyroAngX = angleX - anvY*dt;
			gyroAngY = angleY + anvX*dt;
			angleX = abs((0.98*gyroAngX)+(0.02*accAngX));
			angleY = abs((0.98*gyroAngY)+(0.02*accAngY));
			servoAngY = asin(sin((90-angleX)/180*3.1415)/sin(angleY/180*3.1415))/3.1415*180;
			servoY.SetDegree(Y_DIR_SERVO_MID+servoAngY*8);
			servoAngX = 90-angleY;
			servoX.SetDegree(X_DIR_SERVO_MID-servoAngX*8);

		}

	}
	return 0;
}

