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
#include <vector>
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
#include <libsc/alternate_motor.h>
#include <libsc/mpu6050.h>
#include <libsc/mini_lcd.h>
#include <libsc/joystick.h>
#include <libsc/st7735r.h>
#include <libsc/lcd_typewriter.h>
#include <libutil/math.h>
#include <math.h>

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

#define CAM_W 80
#define CAM_H 60
uint8_t MEAN_FILTER_WINDOW_SIZE = 1;	//window size should be odd

#define ENABLE_LCD

//#define DORNE_ONE

#define DISTANCE_CHECK 30

#define FAR_DISTANCE 10

#define NO_OF_POINTS_OF_REGIONS_FOR_DRONE_TWO 18

#define DRONE_TWO_TRANS_X 3
#define DRONE_TWO_TRANS_Y 3


void imageConversion(bool des[CAM_W][CAM_H], Byte src[CAM_W * CAM_H / 8]);

void imageConversionBack(Byte des[CAM_W * CAM_H / 8], bool src[CAM_W][CAM_H]);

void meanFilter(bool des[CAM_W][CAM_H], bool in[CAM_W][CAM_H]);

void centreFinder(vector<Coor>& cen, bool in[CAM_W][CAM_H]);

void determinePts(vector<Coor>& pt, Coor& beacon, Coor& car);

void multiplePts(vector<Coor>& pts, Coor& Beacon, Coor& Car, int carOnly);

//put centre in for convenience of DRONE TWO
void sendSignal(const vector<Coor>& cen);

bool switchBeacon(Coor bn, Coor Cr);

uint32_t squaredDistance(const Coor& b, const Coor& c);

void translatePoints(vector<Coor>& c);

void excludeCentreFarAwayFromCar(vector<Coor>& c);


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

Coor beacon_old;

bool startTheDroneProcess = false;

bool fakeBeacon = false;
bool beaconCarVeryClose = false;


Coor car;
Coor beacon;
bool startFlag = false;
string messageFromDrone;
vector<int16_t> coorBuffer;


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

#ifdef DRONE_ONE
//			//update preCentre by using data in centre
//			determinePts(centre, beacon, car);

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
#endif


			// //signal from the car to reset the data in the drone
			// if(startTheDroneProcess)
			// {
			// 	beacon = Coor();
			// 	car = Coor();
			// 	startTheDroneProcess = false;
			// }

			//send the two coordinates in preCentre to the Car with the first one the Car, the second one the Beacon
			sendSignal(centre);


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

void imageConversion(bool des[CAM_W][CAM_H], Byte src[CAM_W * CAM_H / 8])
{
	for(uint16_t i = 0; i < CAM_H; i++)
		for(uint16_t j = 0; j < CAM_W / 8; j++)
			for(uint8_t k = 0; k < 8; k++)
				des[j * 8 + k][i] = ( src[i * (CAM_W / 8) + j] >> (7 - k) ) & 1;
}

void imageConversionBack(Byte des[CAM_W * CAM_H / 8], bool src[CAM_W][CAM_H])
{
	for(uint16_t i = 0; i < CAM_H * CAM_W / 8; i++)
		des[i] = 0;

	for(uint16_t i = 0; i < CAM_H; i++)
		for(uint16_t j = 0; j < CAM_W; j++)
			des[ (i * CAM_W + j) / 8 ] = ( src[j][i] << (7 - j % 8) ) | des[(i * CAM_W + j) / 8];
}

void meanFilter(bool des[CAM_W][CAM_H], bool in[CAM_W][CAM_H])
{
	//init a sum array
	uint8_t sum[CAM_W - MEAN_FILTER_WINDOW_SIZE + 1];
	for(uint16_t j = 0; j < CAM_W - MEAN_FILTER_WINDOW_SIZE + 1; j++)
		sum[j] = 0;

	//calculate sum[0]
	for(uint8_t a = 0; a < MEAN_FILTER_WINDOW_SIZE; a++)
		for(uint8_t b = 0; b < MEAN_FILTER_WINDOW_SIZE; b++)
			sum[0] += in[a][b];

	//calculate the rest sum along x direction
	for(uint8_t a = 1; a < CAM_W - MEAN_FILTER_WINDOW_SIZE + 1; a++)
	{
		sum[a] = sum[a - 1];
		for(uint8_t b = 0; b < MEAN_FILTER_WINDOW_SIZE; b++)
			sum[a] = sum[a] - in[a - 1][b] + in[a + MEAN_FILTER_WINDOW_SIZE - 1][b];
	}

	//do filtering
	//iterate throught y direction
	for(uint16_t i = MEAN_FILTER_WINDOW_SIZE / 2; i < CAM_H - MEAN_FILTER_WINDOW_SIZE / 2; i++)
	{
		if(i > MEAN_FILTER_WINDOW_SIZE / 2)
			//update mean for the row
			for (uint16_t k = 0; k < CAM_W - MEAN_FILTER_WINDOW_SIZE + 1; k++)
				for(uint8_t l = 0; l < MEAN_FILTER_WINDOW_SIZE; l++)
					sum[k] = sum[k] - in[k + l][i - 1 - MEAN_FILTER_WINDOW_SIZE / 2]
									+ in[k + l][i + MEAN_FILTER_WINDOW_SIZE / 2];

		//calculate the mean for des[j][i]
		//iterate through x direction
		for(uint16_t j = MEAN_FILTER_WINDOW_SIZE / 2; j < CAM_W - MEAN_FILTER_WINDOW_SIZE / 2; j++)
		{
			des[j][i] = sum[j - MEAN_FILTER_WINDOW_SIZE / 2] > ( ( MEAN_FILTER_WINDOW_SIZE * MEAN_FILTER_WINDOW_SIZE ) / 2 );
//			 uint16_t temp = sum[j - MEAN_FILTER_WINDOW_SIZE / 2] >= ( MEAN_FILTER_WINDOW_SIZE * MEAN_FILTER_WINDOW_SIZE );
//			 assert( temp <= 1);
		}
	}

	//erase boundary
	for(uint16_t i = 0; i < CAM_W; i++)
		for(uint16_t j = 0; j < MEAN_FILTER_WINDOW_SIZE / 2; j++)
			des[i][j] = 1;

	for(uint16_t i = 0; i < CAM_W; i++)
		for(uint16_t j = CAM_H - MEAN_FILTER_WINDOW_SIZE / 2; j < CAM_H; j++)
			des[i][j] = 1;

	for(uint16_t i = 0; i < CAM_H; i++)
		for(uint16_t j = 0; j < MEAN_FILTER_WINDOW_SIZE / 2; j++)
			des[j][i] = 1;

	for(uint16_t i = 0; i < CAM_H; i++)
		for(uint16_t j = CAM_W - MEAN_FILTER_WINDOW_SIZE / 2; j < CAM_W; j++)
			des[j][i] = 1;
}

void centreFinder(vector<Coor>& cen, bool in[CAM_W][CAM_H])
{
	uint8_t distriX[CAM_W];
	uint8_t distriY[CAM_H];
	//init
	for(uint8_t i = 0; i < CAM_W; i++)
		distriX[i] = 0;
	for(uint8_t i = 0; i < CAM_H; i++)
		distriY[i] = 0;
	for(uint8_t i = 0; i < CAM_W; i++)
		for(uint8_t j = 0; j < CAM_H; j++)
			if(in[i][j] == 0)
				distriX[i] += 1;

	for(uint8_t i = 0; i < CAM_H; i++)
		for(uint8_t j = 0; j < CAM_W; j++)
			if(in[j][i] == 0)
				distriY[i] += 1;

//#ifdef ENABLE_LCD
//	for(uint8_t i = 0; i < CAM_W; i++)
//	{
//		lcdP->SetRegion(Lcd::Rect(3 + i, 2 + CAM_H, 1, distriX[i]));
//		lcdP->FillColor(Lcd::kYellow);
//	}
//
//	for(uint8_t i = 0; i < CAM_H; i++)
//	{
//		lcdP->SetRegion(Lcd::Rect(3 + CAM_W, 2 + i, distriY[i], 1));
//		lcdP->FillColor(Lcd::kYellow);
//	}
//
//
//	for(uint8_t i = 0; i < CAM_W; i++)
//	{
//		lcdP->SetRegion(Lcd::Rect(3 + i, 2 + CAM_H, 1, distriX[i]));
//		lcdP->FillColor(Lcd::kBlack);
//	}
//
//	for(uint8_t i = 0; i < CAM_H; i++)
//	{
//		lcdP->SetRegion(Lcd::Rect(3 + CAM_W, 2 + i, distriY[i], 1));
//		lcdP->FillColor(Lcd::kBlack);
//	}
//#endif

	cen.reserve(10);

	for(uint8_t i = 0; i < CAM_H; i++)
	{
		if(distriY[i])
		{
			for(uint8_t j = 0; j < CAM_W; j++)
			{
				if(distriX[j])
				{
					//a white point found
					if(in[j][i] == 0)
					{
						queue<Coor> qForExpand;
						queue<Coor> qForStore;
						qForExpand.push(Coor(j, i));
						qForStore.push(Coor(j, i));
						in[j][i] = 1;
						while(qForExpand.empty()!=true)
						{
							Coor cur = qForExpand.front();
							qForExpand.pop();

							//for near points out of bound checking
							//true if no out of bound
							bool xU = cur.x+1 < CAM_W;
							bool xL = cur.x-1 > 0;
							bool yU = cur.y-1 > 0;
							bool yL = cur.y+1 < CAM_H;

							//check whether the near point is white
							//if yes, push it in startingPoint, clear the corresponding coordinate in white
							//and also set the value in indexImage to 0 to avoid double pushing
							if(xU)
							{
								if(in[cur.x+1][cur.y] == 0)
								{
									//push
									qForExpand.push(Coor(cur.x+1, cur.y));

									//push into the region
									qForStore.push(Coor(cur.x+1, cur.y));
									in[cur.x+1][cur.y] = 1;
								}

								if(yU)
								{
									if(in[cur.x+1][cur.y-1] == 0)
									{
										//push
										qForExpand.push(Coor(cur.x+1, cur.y-1));

										//push into the region
										qForStore.push(Coor(cur.x+1, cur.y-1));
										in[cur.x+1][cur.y-1] = 1;
									}
								}

								if(yL)
								{
									if(in[cur.x+1][cur.y+1] == 0)
									{
										//push
										qForExpand.push(Coor(cur.x+1, cur.y+1));

										//push into the region
										qForStore.push(Coor(cur.x+1, cur.y+1));
										in[cur.x+1][cur.y+1] = 1;
									}
								}
							}

							if(xL)
							{
								if(in[cur.x-1][cur.y] == 0)
								{
									//push
									qForExpand.push(Coor(cur.x-1, cur.y));

									//push into the region
									qForStore.push(Coor(cur.x-1, cur.y));
									in[cur.x-1][cur.y] = 1;
								}

								if(yU)
								{
									if(in[cur.x-1][cur.y-1] == 0)
									{
										//push
										qForExpand.push(Coor(cur.x-1, cur.y-1));

										//push into the region
										qForStore.push(Coor(cur.x-1, cur.y-1));
										in[cur.x-1][cur.y-1] = 1;
									}
								}

								if(yL)
								{
									if(in[cur.x-1][cur.y+1] == 0)
									{
										//push
										qForExpand.push(Coor(cur.x-1, cur.y+1));

										//push into the region
										qForStore.push(Coor(cur.x-1, cur.y+1));
										in[cur.x-1][cur.y+1] = 1;
									}
								}
							}

							if(yU)
							{
								if(in[cur.x][cur.y-1] == 0)
								{
									//push
									qForExpand.push(Coor(cur.x, cur.y-1));

									//push into the region
									qForStore.push(Coor(cur.x, cur.y-1));
									in[cur.x][cur.y-1] = 1;
								}
							}

							if(yL)
							{
								if(in[cur.x][cur.y+1] == 0)
								{
									//push
									qForExpand.push(Coor(cur.x, cur.y+1));

									//push into the region
									qForStore.push(Coor(cur.x, cur.y+1));
									in[cur.x][cur.y+1] = 1;
								}
							}
						}//end of while loop for expanding the region


						uint64_t cenX = 0;
						uint64_t cenY = 0;
						uint32_t base = qForStore.size();
#ifndef DRONE_ONE
						if(qForStore.size() > NO_OF_POINTS_OF_REGIONS_FOR_DRONE_TWO)
							goto END_OF_IF_ONE_WHITE_POINT_IS_FOUND;
#endif

						for(uint16_t k = 0; k < base; k++)
						{
							Coor tempCoor = qForStore.front();
							qForStore.pop();
							cenX += tempCoor.x;
							cenY += tempCoor.y;
						}
						cen.push_back(Coor(cenX/base, cenY/base));
END_OF_IF_ONE_WHITE_POINT_IS_FOUND:
						;
					}//end of if one white point is found
				}
			}
		}
	}//end of centrefinding for loop

}

void determinePts(vector<Coor>& pt, Coor& beacon, Coor& car)
{
	uint32_t d1, d2, carD1, carD2;

	uint16_t centreNo = pt.size();

	// 1st Run
	if (runFlag == 0)
	{
		// beacon is found
		if (centreNo == 1)
		{
			prevCentreCount = centreNo;
			beacon_old = beacon = pt[0];
			runFlag = 1;
		}
			//		else if (centreCount > 1)
			//		{	centreStorage = centre;	}
	}
	// 2nd Run/ beacon is already found
	else if (runFlag == 1)
	{
		if (centreNo > 0)
		{
			// Calculate beacon & new pt dist
			d1 = (beacon.x - pt[0].x) * (beacon.x - pt[0].x) + (beacon.y - pt[0].y) * (beacon.y - pt[0].y);
			beacon_old = beacon;
		}

		//	Last time only 1 centre is found
		if (prevCentreCount == 1)
		{
			//	This time also only 1 centre is found
			if (centreNo == 1)
			{
					if (switchBeacon(beacon, car) == false)
					{
						// Update beacon Coor
						beacon = pt[0];
					}
			}

			// This time 2 centres are located
			else if (centreNo == 2)
			{
				//	Compare beacon DISTANCE_CHECKance with 2 located centres

				if (switchBeacon(beacon, car) == false)
				{
					d2 = (beacon.x - pt[1].x) * (beacon.x - pt[1].x) + (beacon.y - pt[1].y) * (beacon.y - pt[1].y);

					if ( d1 > d2 )					//	Swap occurs
					{
						if ( d2 < (DISTANCE_CHECK * DISTANCE_CHECK))
						{
							beacon = pt[1];
							car = pt[0];
						}
					}else if ( d1 < (DISTANCE_CHECK * DISTANCE_CHECK))
					{
						car = pt[1];
						beacon = pt[0];
					}
				}
				else					// Update new beacon position
				{
					carD1 = (car.x - pt[0].x) * (car.x - pt[0].x) + (car.y - pt[0].y) * (car.y - pt[0].y);
					carD2 = (car.x - pt[1].x) * (car.x - pt[1].x) + (car.y - pt[1].y) * (car.y - pt[1].y);
					if ( carD1 > carD2 )					//	Swap occurs
					{
						car = pt[1];
						beacon = pt[0];
					}
					else
					{
						beacon = pt[1];
						car = pt[0];
					}

				}

		//		carAngle = angleTracking(beacon, car);
		//		carTurnAngle = carAngle + carTurnAngle ;
			}

			// This time >2 centres
			else if (centreNo > 2)
			{
				// Only compare and update beacon Coor with all the ponts
				if (switchBeacon(beacon, car) == false)
				{	multiplePts(pt, beacon, car, 1);	}
			}
		}		// End Last time only 1 centre is found

		//	Last time 2 centres are found
		else if ( prevCentreCount == 2 )
		{
			// This time only 1 centre -> update beacon
			if (centreNo == 1)
			{
					if (switchBeacon(beacon, car) == false)	//((d1 < (DISTANCE_CHECK * DISTANCE_CHECK)) &&
					{
						// Update beacon Coor
						beacon = pt[0];
					}
			}
			// More than 1 pts -> Update both beacon and car
			else if (centreNo > 1)
			{
				multiplePts(pt, beacon, car, 0);
			}
		}		// End Last time 2 centres are found

		//	Multiple pt -> 2 centres
		prevCentreCount = centreNo;
		if (prevCentreCount > 1)
		{	prevCentreCount = 2;	}
		//	Ignored case: No centre is found
		else if (prevCentreCount == 0)
		{	prevCentreCount = 1;	}

/*		if ((centreNo > 0 ) && (switchBeacon(beacon, car) == false))
		{
			droneDriftAngle = angleTracking(beacon_old, beacon) -  droneDriftAngle;
		}
*/
	}		// end Run flag = 1
}

void multiplePts(vector<Coor>& pts, Coor& Beacon, Coor& Car, int carOnly)
{
	uint16_t cCount = pts.size();
	int tempResult, tempIndex = 0;
	int smallestD = DISTANCE_CHECK * DISTANCE_CHECK;		// temp storage for smallest dist.

	//	Compare beacon Coor with all pts
	for (int i = 0; i < (cCount-1); i++)
	{
		tempResult = (Beacon.x - pts[i].x)*(Beacon.x - pts[i].x) + (Beacon.y - pts[i].y) * (Beacon.y - pts[i].y);
		if (tempResult < smallestD)
		{
			smallestD = tempResult;
			tempIndex = i;
		}
	}

	if ((tempIndex > 0)&& (smallestD < DISTANCE_CHECK* DISTANCE_CHECK))			// shortest D is found
	{
		Beacon.x = pts[tempIndex].x;
		Beacon.y = pts[tempIndex].y;
	}

	//	Compare car Coor with all pts
	if (carOnly != 1)
	{
		tempIndex = 0;
		smallestD = DISTANCE_CHECK * DISTANCE_CHECK;

		for (int j = 0; j < (cCount-1); j++)
		{
			tempResult = (Car.x - pts[j].x)*(Car.x - pts[j].x) + (Car.y - pts[j].y) * (Car.y - pts[j].y);
			if (tempResult < smallestD)
			{
				smallestD = tempResult;
				tempIndex = j;
			}
		}
		if (tempIndex > 0) //&& (smallestD < DISTANCE_CHECK* DISTANCE_CHECK))			// shortest D is found
		{
			Car.x = pts[tempIndex].x;
			Car.y = pts[tempIndex].y;
		}
	}
}

bool switchBeacon(Coor bn, Coor Cr)
{
	if (((Cr.x- bn.x) * (Cr.x- bn.x) + (Cr.y- bn.y) * (Cr.y- bn.y)) < (8 * 8))
	{	return true; }
	else
	{ return false;	}
}

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

void sendSignal(const vector<Coor>& cen)
{
	string tempMessage;
	ledP[0]->Switch();
	tempMessage += 's';

	char buffer[20];

	if(beacon == Coor())
	{
		tempMessage += "-01,";
		tempMessage += "-01,";
	}else
	{
		if(beacon.x <= 9)
		{
			sprintf(buffer, "00%d,", beacon.x);
		}else if(beacon.x <= 99)
		{
			sprintf(buffer, "0%d,", beacon.x);
		}else
			sprintf(buffer, "%d,", beacon.x);

		tempMessage += buffer;

		if(beacon.y <= 9)
		{
			sprintf(buffer, "00%d,", beacon.y);
		}else if(beacon.y <= 99)
		{
			sprintf(buffer, "0%d,", beacon.y);
		}else
		{
			sprintf(buffer, "%d,", beacon.y);
		}

		tempMessage += buffer;
	}
	if(car == Coor())
	{
		tempMessage += "-01,";
		tempMessage += "-01,";
	}else
	{
		if(car.x <= 9)
		{
			sprintf(buffer, "00%d,", car.x);
		}else if(car.x <= 99)
		{
			sprintf(buffer, "0%d,", car.x);
		}else
		{
			sprintf(buffer, "%d,", car.x);
		}

		tempMessage += buffer;

		if(car.y <= 9)
		{
			sprintf(buffer, "00%d,", car.y);
		}else if(car.y <= 99)
		{
			sprintf(buffer, "0%d,", car.y);
		}else
		{
			sprintf(buffer, "%d,", car.y);
		}

		tempMessage += buffer;
	}

#ifndef DRONE_ONE
	//for obstacle detection
	for(vector<Coor>::const_iterator it = cen.begin(); it != cen.end(); it++ )
	{
		if(it->x <= 9)
		{
			sprintf(buffer, "00%d,", it->x);
		}else if(beacon.x <= 99)
		{
			sprintf(buffer, "0%d,", it->x);
		}else
			sprintf(buffer, "%d,", it->x);

		tempMessage += buffer;

		if(it->y <= 9)
		{
			sprintf(buffer, "00%d,", it->y);
		}else if(beacon.y <= 99)
		{
			sprintf(buffer, "0%d,", it->y);
		}else
		{
			sprintf(buffer, "%d,", it->y);
		}

		tempMessage += buffer;
	}
#endif

	tempMessage += 'e';
	bluetoothP->SendStr(tempMessage);
}

uint32_t squaredDistance(const Coor& b, const Coor& c)
{
	int32_t xCom = b.x - c.x;
	int32_t yCom = b.y - c.y;
	return ( xCom * xCom ) + ( yCom * yCom );
}

void translatePoints(vector<Coor>& c)
{
	for(vector<Coor>::iterator it = c.begin(); it != c.end(); it++)
	{
		it->x += DRONE_TWO_TRANS_X;
		it->y += DRONE_TWO_TRANS_Y;
	}
}

k60::Ov7725::Config getCameraConfig()
{
	k60::Ov7725::Config c;
	c.id = 0;
	c.w = CAM_W;
	c.h = CAM_H;
	c.fps = k60::Ov7725Configurator::Config::Fps::kMid;

	c.contrast = 0x10;
	c.brightness = 0x43;
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

#ifdef DORNE_ONE
bool bluetoothListener(const Byte* data, const size_t size)
{
	if(*data == 'g')
	{
		startTheDroneProcess = true;
		bluetoothP->SendStr("g");
	}
	return true;
}
#endif

#ifndef DORNE_ONE
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
			beacon = Coor(coorBuffer[0], coorBuffer[1]);
			car = Coor(coorBuffer[2], coorBuffer[3]);
			coorBuffer.clear();
			startFlag = false;

			//update the beacon coordinate by history
//			carLocate();
		}else if(*data == ',')
		{
			int32_t tempCoordinate = stoi(messageFromDrone);
			coorBuffer.push_back(tempCoordinate);
			messageFromDrone.clear();
		}else if(startFlag == true)
		{
			messageFromDrone += *data;
		}
	}
	return true;
}
#endif

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
	c.gyro_range = Mpu6050::Config::Range::kMid;
	c.accel_range = Mpu6050::Config::Range::kMid;
	return c;
}
