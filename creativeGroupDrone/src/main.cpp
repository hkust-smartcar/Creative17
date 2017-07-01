/*
 * main.cpp
 *
 * Author: Gordon
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <cassert>
#include <cstring>
#include <vector>
#include <queue>
#include <cstdint>
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
#include <libsc/alternate_motor.h>
#include <libsc/mpu6050.h>
#include <libsc/mini_lcd.h>
#include <libsc/joystick.h>
#include <libsc/st7735r.h>
#include <libsc/lcd_typewriter.h>
#include <libutil/math.h>
//#include <math.h>

#include "Coor.h"
#include "PointProcessing.h"
#include "ImageProcessing.h"

using namespace std;

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

//
//#define CAM_W 80
//#define CAM_H 60
//uint8_t MEAN_FILTER_WINDOW_SIZE = 5;	//window size should be odd

//#define ENABLE_LCD

#define DISTANCE_CHECK 30

void sendSignal(const Coor& b, const Coor& c);

k60::Ov7725::Config getCameraConfig();
UartDevice::Config getBluetoothConfig();
Mpu6050::Config getMpuConfig();
St7735r::Config getLcdConfig();
Joystick::Config getJoystickConfig(uint8_t _id);


//void printCameraImage(const Byte* image);
bool bluetoothListener(const Byte* data, const size_t size);


Led* ledP[4];
St7735r* lcdP;
Ov7725* cameraP;
JyMcuBt106* bluetoothP;
Joystick* joystickP;
LcdTypewriter* writerP;
bool runFlag = 0;
uint8_t prevCentreCount = 0;

bool startTheDroneProcess = false;

bool fakeBeacon = false;
bool beaconCarVeryClose = false;


///////////////////////////////////////////////////////This is for the drone
int main(void)
{
	System::Init();

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
	config.lcd = &lcd; // St7735r lcd from prevCentreCount page
	LcdTypewriter writer(config);
	writerP = &writer;

	//--------------------------------bluetooth
	JyMcuBt106 bt(getBluetoothConfig());
	bluetoothP = &bt;

	//--------------------------------camera
	Ov7725 camera(getCameraConfig());
	cameraP = &camera;

	//data
	Byte cameraBuffer[CAM_W * CAM_H / 8];
	bool boolImage[CAM_W][CAM_H];
	bool boolImageFiltered[CAM_W][CAM_H];
	vector<Coor> centre;
//	vector<Coor> preCentre;		//assume size of two
	Coor beacon;
	Coor car;

	//initialize camera related arrays and vectors
	memset(cameraBuffer, 0, CAM_W * CAM_H / 8);
	for(int i = 0; i < CAM_W; i++)
		memset(boolImage + i, 0, CAM_H);
//	preCentre.push_back(Coor());
//	preCentre.push_back(Coor());

	cameraP->Start();

	//--------------------------------JOYSTICK
	Joystick js(getJoystickConfig(0));

//	while(!startTheDroneProcess)
//	{
//		System::DelayMs(5);
//	}
//
//	//spare for next reset
//	startTheDroneProcess = false;

	uint32_t lastTime = System::Time();
	while(true)
	{
		if( ( System::Time() - lastTime ) >= 50)
		{
			lastTime = System::Time();

			//update camera image
			memcpy(cameraBuffer, cameraP->LockBuffer(), CAM_W * CAM_H / 8);
			cameraP->UnlockBuffer();

//			//print camera image easy version
//			lcdP->SetRegion(Lcd::Rect(3, 2, CAM_W, CAM_H));
//			//0 = white ************** in camera image
//			//White = 0xFFFF = white in real
//			lcdP->FillBits(0x0000, 0xFFFF, cameraBuffer, CAM_W * CAM_H);

			//formatting to boolean form
			imageConversion(boolImage, cameraBuffer);

			//filtering
			meanFilter(boolImageFiltered, boolImage);

			//formatting to Byte from
			imageConversionBack(cameraBuffer, boolImageFiltered);

#ifdef ENABLE_LCD
			//print camera image easy version
			lcdP->SetRegion(Lcd::Rect(3, 2, CAM_W, CAM_H));
			//0 = white ************** in camera image
			//White = 0xFFFF = white in real
			lcdP->FillBits(0x0000, 0xFFFF, cameraBuffer, CAM_W * CAM_H);
#endif

			//find centre of white regions
			centreFinder(centre, boolImageFiltered);

#ifdef ENABLE_LCD
			//for print no. of centre out
			lcdP->SetRegion(Lcd::Rect(0,100,128,50));
			char buffer[50];
			sprintf(buffer, "no.: %d   %d", centre.size(), MEAN_FILTER_WINDOW_SIZE);
			writerP->WriteString(buffer);
#endif

			//indicate the centre of white regions with red little square
			if(centre.size()>0)
			{
				for(uint8_t a = 0; a < centre.size(); a++)
				{
					if(a>0)
						assert(centre[a]!=centre[a-1]);
#ifdef ENABLE_LCD
					lcdP->SetRegion(Lcd::Rect(3 + centre[a].x, 2 + centre[a].y, 2, 2));
					lcdP->FillColor(St7735r::kRed);
#endif
				}
			}

//			//update preCentre by using data in centre
//			determinePts(centre, beacon, car, prevCentreCount, runFlag);

#define FAR_DISTANCE 10
			if(beaconCarVeryClose == true && centre.size() == 0)
			{
				//STEP 5: the car got to the beacon but the new beacon is too far away such that it is not in the frame
				beaconCarVeryClose = false;
				car = beacon;
				beacon = Coor(CAM_W / 2, CAM_H / 2);
				fakeBeacon = true;
			}else if(beaconCarVeryClose == true && centre.size() == 1)
			{

				//STEP 5: the car got to the beacon and the new beacon is near by
				if(squaredDistance(beacon, centre[0]) >= FAR_DISTANCE * FAR_DISTANCE)
				{
					beaconCarVeryClose = false;
					car = beacon;
					beacon = centre[0];
				}
			}else if(beaconCarVeryClose == true && centre.size() == 2)
			{
				//don't know what to do

				// //the distane become large again. Need to recalculate the angle so update the coordinate
				// if(squaredDistance(centre[0], centre[1]) > FAR_DISTANCE * FAR_DISTANCE)
				// {
				// 	//choose the closest point to the beacon to be the new beacon
				// 	if(squaredDistance(beacon, centre[0]) < squaredDistance(beacon, centre[1]))
				// 	{
				// 		beacon = centre[0];
				// 		car = centre[1];
				// 	}else
				// 	{
				// 		beacon = centre[1];
				// 		car = centre[0];
				// 	}
				// 	beaconCarVeryClose = false;
				// }

			}else if(centre.size() == 1)
			{
				//STEP 1:
				//assume the starting point is car if beacon is invalid
				if(beacon == Coor())
				{
					car = centre.front();
					beacon = Coor(CAM_W / 2, CAM_H / 2);
					fakeBeacon = true;
				}else if(fakeBeacon == false)
				{
					beacon = centre.front();
				}else if(squaredDistance(beacon, car) >= FAR_DISTANCE * FAR_DISTANCE) //STEP 4: the car approches the beacon
				{
					beacon = centre.front();
					//don't know any information about the car
					//hold that first
					beaconCarVeryClose = false;
				}else //else do not update the beacon coordinate to avoid blinking
				{
					beaconCarVeryClose = true;
				}
			}else if(centre.size() == 2)
			{
				//STEP 2:
				if(fakeBeacon == true)
				{
					if(squaredDistance(car, centre[0]) < squaredDistance(car, centre[1]))
					{
						car = centre[0];
						beacon = centre[1];
					}else
					{
						car = centre[1];
						beacon = centre[0];
					}
					fakeBeacon = false;
				}else //STEP 3: update the beacon first
				{
					//choose the closest point to the beacon to be the new beacon
					if(squaredDistance(beacon, centre[0]) < squaredDistance(beacon, centre[1]))
					{
						beacon = centre[0];
						car = centre[1];
					}else
					{
						beacon = centre[1];
						car = centre[0];
					}
				}

			}


			// //signal from the car to reset the data in the drone
			// if(startTheDroneProcess)
			// {
			// 	beacon = Coor();
			// 	car = Coor();
			// 	startTheDroneProcess = false;
			// }

			//send the two coordinates in preCentre to the Car with the first one the Car, the second one the Beacon
			sendSignal(beacon, car);


#ifdef ENABLE_LCD
			lcdP->SetRegion(Lcd::Rect(3 + beacon.x, 2 + beacon.y, 3, 3));
			lcdP->FillColor(St7735r::kBlue);
			lcdP->SetRegion(Lcd::Rect(3 + car.x, 2 + car.y, 3, 3));
			lcdP->FillColor(St7735r::kRed);

			lcdP->SetRegion(Lcd::Rect(0, 120, 128, 50));
			sprintf(buffer, "beacon B %d %d", beacon.x, beacon.y);
			writerP->WriteString(buffer);

			lcdP->SetRegion(Lcd::Rect(0, 140, 128, 50));
			sprintf(buffer, "car G %d %d", car.x, car.y);
			writerP->WriteString(buffer);
#endif

			centre.clear();


		}//end if for checking time
	}//end while loop
	return 0;
}

//
//
//


float angleTracking(Coor beacon1, Coor beacon2)
{
	// Whole circle bearing
	int deltaX = beacon2.x - beacon1.x;
	int deltaY = beacon2.y - beacon1.y;

	if ((deltaX != 0) && (deltaY != 0))
	{
		// The degree that the image has rotated
		float angle = 10 * Math::ArcTan(abs(deltaY)/abs(deltaX));
		if (deltaX > 0)
		{
			if (deltaY > 0)
			{	return (90 - angle);	}
			else
			{	return (90 + angle);	}
		}
		else
		{
			if (deltaY > 0)
			{	return (-90 + angle);	}
			else
			{	return (-90 - angle);	}
		}
	}
	else
	{	return 0;	}
}

void sendSignal(const Coor& b, const Coor& c)
{
	string tempMessage;
	ledP[0]->Switch();
	tempMessage += 's';

	char buffer[20];

	if(b == Coor())
	{
		tempMessage += "-01,";
		tempMessage += "-01,";
	}else
	{
		if(b.x <= 9)
		{
			sprintf(buffer, "00%d,", b.x);
		}else if(b.x <= 99)
		{
			sprintf(buffer, "0%d,", b.x);
		}else
			sprintf(buffer, "%d,", b.x);

		tempMessage += buffer;

		if(b.y <= 9)
		{
			sprintf(buffer, "00%d,", b.y);
		}else if(b.y <= 99)
		{
			sprintf(buffer, "0%d,", b.y);
		}else
		{
			sprintf(buffer, "%d,", b.y);
		}

		tempMessage += buffer;
	}
	if(c == Coor())
	{
		tempMessage += "-01,";
		tempMessage += "-01,";
	}else
	{
		if(c.x <= 9)
		{
			sprintf(buffer, "00%d,", c.x);
		}else if(c.x <= 99)
		{
			sprintf(buffer, "0%d,", c.x);
		}else
		{
			sprintf(buffer, "%d,", c.x);
		}

		tempMessage += buffer;

		if(c.y <= 9)
		{
			sprintf(buffer, "00%d,", c.y);
		}else if(c.y <= 99)
		{
			sprintf(buffer, "0%d,", c.y);
		}else
		{
			sprintf(buffer, "%d,", c.y);
		}

		tempMessage += buffer;
	}

	tempMessage += 'e';
	bluetoothP->SendStr(tempMessage);
}

//

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
	c.rx_irq_threshold = 1;
	c.rx_isr = bluetoothListener;
	return c;
}

bool bluetoothListener(const Byte* data, const size_t size)
{
	if(*data == 'g')
	{
		startTheDroneProcess = true;
		bluetoothP->SendStr("g");
	}
	return true;
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
//			MEAN_FILTER_WINDOW_SIZE++;
		};
	c.handlers[1] = [](const uint8_t id, const Joystick::State which)
		{
//			MEAN_FILTER_WINDOW_SIZE--;
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
	c.gyro_range = Mpu6050::Config::Range::kMid;
	c.accel_range = Mpu6050::Config::Range::kMid;
	return c;
}
