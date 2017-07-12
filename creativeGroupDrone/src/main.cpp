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


//#define CAM_W 80
//#define CAM_H 60

#define CAM_W 160
#define CAM_H 120
uint8_t MEAN_FILTER_WINDOW_SIZE = 3;	//window size should be odd

#define ENABLE_LCD

#define DRONE_ONE

#define DISTANCE_CHECK 30

// #define FAR_DISTANCE 8

#define NO_OF_POINTS_OF_REGIONS_FOR_DRONE_TWO 18

#define DRONE_TWO_TRANS_X 3
#define DRONE_TWO_TRANS_Y 3
#define CLOSE_TO_BEACON_DISTANCE 13


void imageConversion(bool des[CAM_W][CAM_H], const Byte src[CAM_W * CAM_H / 8]);

void imageConversionBack(Byte des[CAM_W * CAM_H / 8], bool src[CAM_W][CAM_H]);

void meanFilter(bool des[CAM_W][CAM_H], bool in[CAM_W][CAM_H]);

void centreFinder(vector<Coor>& cen, bool in[CAM_W][CAM_H]);

void determinePts(vector<Coor>& pts, Coor& carH, Coor& carT, Coor& beacon);

float angleTracking(Coor carT, Coor carH);

void assignDirection(Coor& newH, Coor& newT, Coor& carH, Coor& carT, float originalAngle);

//put centre in for convenience of DRONE TWO
void sendSignal(const Coor& b, const Coor& cH, const Coor& cT);

void appendSingal(string& singal, const Coor& coordinate);

bool switchBeacon(Coor bn, Coor Cr);

uint32_t squaredDistance(const Coor& b, const Coor& c);

void translatePoints(vector<Coor>& c);

void excludeCentreFarAwayFromCar(vector<Coor>& c);

void boundAngle(float& angle);


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

bool startFlag = false;
string messageFromDrone;
vector<int16_t> coorBuffer;
vector<uint16_t> regionSize;

const Byte* cameraBuffer;
//Byte cameraBufferFiltered[CAM_W * CAM_H / 8];
bool boolImage[CAM_W][CAM_H];
//bool boolImageFiltered[CAM_W][CAM_H];


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
	vector<Coor> centre;
//	vector<Coor> preCentre;		//assume size of two
	Coor beacon;
	Coor carH;
	Coor carT;

//	//initialize camera related arrays and vectors
//	memset(cameraBuffer, 0, CAM_W * CAM_H / 8);
//	for(int i = 0; i < CAM_W; i++)
//		memset(boolImage + i, 0, CAM_H);
//	preCentre.push_back(Coor());
//	preCentre.push_back(Coor());

	cameraP->Start();

	//--------------------------------JOYSTICK
	Joystick js(getJoystickConfig(0));

	while(!startTheDroneProcess)
	{
		ledP[2]->Switch();
		System::DelayMs(20);
	}

	ledP[2]->SetEnable(false);

//	//spare for next reset
//	startTheDroneProcess = false;

	uint32_t lastTime = System::Time();
	while(true)
	{
		if( ( System::Time() - lastTime ) >= 50)
		{
			lastTime = System::Time();

			//update camera image
			cameraBuffer = cameraP->LockBuffer();
			cameraP->UnlockBuffer();

			//formatting to boolean form
			imageConversion(boolImage, cameraBuffer);

//			//filtering
//			meanFilter(boolImageFiltered, boolImage);
//
//			//formatting to Byte from
//			imageConversionBack(cameraBufferFiltered, boolImageFiltered);

#ifdef ENABLE_LCD
//			//print camera image easy version
//			lcdP->SetRegion(Lcd::Rect(3, 2, CAM_W, CAM_H));
//			//0 = white ************** in camera image
//			//White = 0xFFFF = white in real
//			lcdP->FillBits(0x0000, 0xFFFF, cameraBuffer, CAM_W * CAM_H);
#endif //end of ENABLE_LCD

			//find centre of white regions
			centreFinder(centre, boolImage);

#ifdef ENABLE_LCD
			//for print no. of centre out
			lcdP->SetRegion(Lcd::Rect(0,100,128,50));
			char buffer[50];
			sprintf(buffer, "no.: %d   %d", centre.size(), MEAN_FILTER_WINDOW_SIZE);
			writerP->WriteString(buffer);
#endif //end of ENABLE_LCD

// 			//indicate the centre of white regions with red little square
// 			if(centre.size()>0)
// 			{
// 				for(uint8_t a = 0; a < centre.size(); a++)
// 				{
// 					if(a>0)
// 						assert(centre[a]!=centre[a-1]);
// #ifdef ENABLE_LCD
// 					lcdP->SetRegion(Lcd::Rect(3 + centre[a].x, 2 + centre[a].y, 2, 2));
// 					lcdP->FillColor(St7735r::kRed);
// #endif //end of ENABLE_LCD
// 				}
// 			}

#ifdef DRONE_ONE

			determinePts(centre, carH, carT, beacon);

			// if(beaconCarVeryClose == true && centre.size() == 0)
			// {
			// 	//STEP 5: the car got to the beacon but the new beacon is too far away such that it is not in the frame
			// 	beaconCarVeryClose = false;
			// 	car = beacon;
			// 	beacon = Coor(CAM_W / 2, CAM_H / 2);
			// 	fakeBeacon = true;
			// }else if(beaconCarVeryClose == true && centre.size() == 1)
			// {

			// 	//STEP 5: the car got to the beacon and the new beacon is near by
			// 	if(squaredDistance(beacon, centre[0]) >= FAR_DISTANCE * FAR_DISTANCE)
			// 	{
			// 		beaconCarVeryClose = false;
			// 		car = beacon;
			// 		beacon = centre[0];
			// 	}
			// }else if(beaconCarVeryClose == true && centre.size() == 2)
			// {
			// 	//don't know what to do

			// 	// //the distane become large again. Need to recalculate the angle so update the coordinate
			// 	// if(squaredDistance(centre[0], centre[1]) > FAR_DISTANCE * FAR_DISTANCE)
			// 	// {
			// 	// 	//choose the closest point to the beacon to be the new beacon
			// 	// 	if(squaredDistance(beacon, centre[0]) < squaredDistance(beacon, centre[1]))
			// 	// 	{
			// 	// 		beacon = centre[0];
			// 	// 		car = centre[1];
			// 	// 	}else
			// 	// 	{
			// 	// 		beacon = centre[1];
			// 	// 		car = centre[0];
			// 	// 	}
			// 	// 	beaconCarVeryClose = false;
			// 	// }

			// }else if(centre.size() == 1)
			// {
			// 	//STEP 1:
			// 	//assume the starting point is car if beacon is invalid
			// 	if(beacon == Coor())
			// 	{
			// 		car = centre.front();
			// 		beacon = Coor(CAM_W / 2, CAM_H / 2);
			// 		fakeBeacon = true;
			// 	}else if(fakeBeacon == false)
			// 	{
			// 		beacon = centre.front();
			// 	}else if(squaredDistance(beacon, car) >= FAR_DISTANCE * FAR_DISTANCE) //STEP 4: the car approches the beacon
			// 	{
			// 		beacon = centre.front();
			// 		//don't know any information about the car
			// 		//hold that first
			// 		beaconCarVeryClose = false;
			// 	}else //else do not update the beacon coordinate to avoid blinking
			// 	{
			// 		beaconCarVeryClose = true;
			// 	}
			// }else if(centre.size() == 2)
			// {
			// 	//STEP 2:
			// 	if(fakeBeacon == true)
			// 	{
			// 		if(squaredDistance(car, centre[0]) < squaredDistance(car, centre[1]))
			// 		{
			// 			car = centre[0];
			// 			beacon = centre[1];
			// 		}else
			// 		{
			// 			car = centre[1];
			// 			beacon = centre[0];
			// 		}
			// 		fakeBeacon = false;
			// 	}else //STEP 3: update the beacon first
			// 	{
			// 		//choose the closest point to the beacon to be the new beacon
			// 		if(squaredDistance(beacon, centre[0]) < squaredDistance(beacon, centre[1]))
			// 		{
			// 			beacon = centre[0];
			// 			car = centre[1];
			// 		}else
			// 		{
			// 			beacon = centre[1];
			// 			car = centre[0];
			// 		}
			// 	}

			// }
#endif //end of DRONE_ONE


			// //signal from the car to reset the data in the drone
			// if(startTheDroneProcess)
			// {
			// 	beacon = Coor();
			// 	car = Coor();
			// 	startTheDroneProcess = false;
			// }

			//send the two coordinates in preCentre to the Car with the first one the Car, the second one the Beacon
			sendSignal(beacon, carH, carT);


#ifdef ENABLE_LCD
//			lcdP->SetRegion(Lcd::Rect(3 + beacon.x, 2 + beacon.y, 3, 3));
//			lcdP->FillColor(St7735r::kPurple);
//			lcdP->SetRegion(Lcd::Rect(3 + carH.x, 2 + carH.y, 3, 3));
//			lcdP->FillColor(St7735r::kBlue);
//			lcdP->SetRegion(Lcd::Rect(3 + carT.x, 2 + carT.y, 3, 3));
//			lcdP->FillColor(St7735r::kRed);

			lcdP->SetRegion(Lcd::Rect(0, 115, 128, 50));
			sprintf(buffer, "Bea: %d %d\nCH:\t%d %d\nCT:\t%d %d", beacon.x, beacon.y, carH.x, carH.y, carT.x, carT.y);
			writerP->WriteString(buffer);
#endif //end of ENABLE_LCD

			centre.clear();


		}//end if for checking time
	}//end while loop
	return 0;
}

void imageConversion(bool des[CAM_W][CAM_H], const Byte src[CAM_W * CAM_H / 8])
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
						regionSize.push_back(base);
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

void determinePts(vector<Coor>& pts, Coor& carH, Coor& carT, Coor& beacon)
{
	uint8_t cCount = pts.size();

	//three point case
	if (cCount == 3)
	{
		uint16_t d1 = squaredDistance(pts[0], pts[1]);
		uint16_t d2 = squaredDistance(pts[0], pts[2]);
		uint16_t d3 = squaredDistance(pts[1], pts[2]);

		uint16_t shortestD = d1;
		uint16_t greatestD = d1;

		//find the minimum distance
		if (shortestD > d2)
		{
			shortestD = d2;
		}
		if (shortestD > d3)
		{
			shortestD = d3;
		}

		//find the greatest distance
		if (greatestD < d2)
		{
			greatestD = d2;
		}
		if (greatestD < d3)
		{
			greatestD = d3;
		}

		//do not update the pts if the three are very close to each other
		if (greatestD > CLOSE_TO_BEACON_DISTANCE)
		{
			//determine car head and tail by area
			if (shortestD == d1)
			{
				if (regionSize[0] < regionSize[1])
				{
					carH = pts[0];
					carT = pts[1];
				}else
				{
					carH = pts[1];
					carT = pts[0];
				}
				beacon = pts[2];
			} else if (shortestD == d2)
			{
				if (regionSize[0] < regionSize[2])
				{
					carH = pts[0];
					carT = pts[2];
				}else
				{
					carH = pts[2];
					carT = pts[0];
				}
				beacon = pts[1];
			} else
			{
				if (regionSize[1] < regionSize[2])
				{
					carH = pts[1];
					carT = pts[2];
				}else
				{
					carH = pts[2];
					carT = pts[1];
				}
				beacon = pts[0];
			}
		}

	//two point case
	} else if (cCount == 2)
	{
		//if the two point are close, set them to be the car
		if (squaredDistance(pts[0], pts[1]) < 13 * 13)
		{
			if (regionSize[0] < regionSize[1])
			{
				carH = pts[0];
				carT = pts[1];
			}else
			{
				carH = pts[1];
				carT = pts[0];
			}
		}

		//for guiding the car
		beacon = Coor(CAM_W / 2, CAM_H / 2);
	}
	regionSize.clear();
}

void assignDirection(Coor& newH, Coor& newT, Coor& carH, Coor& carT, float originalAngle)
{
	float angle1 = angleTracking(newT, newH);
	float angle2 = angleTracking(newH, newT);

	angle1 = originalAngle - angle1;
	angle2 = originalAngle - angle2;

	boundAngle(angle1);
	boundAngle(angle2);

	if (angle1 < angle2)
	{
		carH = newH;
		carT = newT;
	}else
	{
		carH = newT;
		carT = newH;
	}
}

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
			return 180;
		} else
		{	
			return 180;
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

void sendSignal(const Coor& b, const Coor& cH, const Coor& cT)
{
	string tempMessage;
	ledP[0]->Switch();

	tempMessage += 's';
	appendSingal(tempMessage, b);
	appendSingal(tempMessage, cH);
	appendSingal(tempMessage, cT);
	tempMessage += 'e';

	bluetoothP->SendStr(tempMessage);
}

void appendSingal(string& singal, const Coor& coordinate)
{
	char buffer[20];
	if(coordinate == Coor())
	{
		singal += "-01,";
		singal += "-01,";
	}else
	{
		if(coordinate.x <= 9)
		{
			sprintf(buffer, "00%d,", coordinate.x);
		}else if(coordinate.x <= 99)
		{
			sprintf(buffer, "0%d,", coordinate.x);
		}else
		{
			sprintf(buffer, "%d,", coordinate.x);
		}

		singal += buffer;

		if(coordinate.y <= 9)
		{
			sprintf(buffer, "00%d,", coordinate.y);
		}else if(coordinate.y <= 99)
		{
			sprintf(buffer, "0%d,", coordinate.y);
		}else
		{
			sprintf(buffer, "%d,", coordinate.y);
		}

		singal += buffer;
	}
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

//	c.contrast = 0x10;
//	c.brightness = 0x43;

//	c.contrast = 0x40;



//	c.brightness = 0x30;
//
//	c.contrast = 0x4c;
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

#ifdef DRONE_ONE
bool bluetoothListener(const Byte* data, const size_t size)
{
	if(*data == 'g')
	{
		startTheDroneProcess = true;
		bluetoothP->SendStr("g");
//		beacon = Coor();
//		fakeBeacon = false;
//		beaconCarVeryClose = false;
	}
	return true;
}
#endif

#ifndef DRONE_ONE
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
