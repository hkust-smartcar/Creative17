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
#include "Coor.h"
#include "img.h"

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
#include <cmath>

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

#define CAM_W 240
#define CAM_H 180
#define CAM_W2 80
#define CAM_H2 60

uint8_t MEAN_FILTER_WINDOW_SIZE = 3;	//window size should be odd

#define ENABLE_LCD

#define DISTANCE_CHECK 30
/*
void getWorldBit(bool destImage[CAM_W2][CAM_H2], bool rawImage[CAM_W][CAM_H])
{
	for (int y = 0; y <CAM_H2; y++)
	{
		for (int x = 0; x <CAM_W2; x++)
		{	destImage[x][y] = rawImage[transformMatrix[x][y][0]][transformMatrix[x][y][1]];	}
	}
}

*/

void imageConversion(bool des[CAM_W][CAM_H], Byte src[CAM_W * CAM_H / 8]);
void imageConversionBack(Byte des[CAM_W2 * CAM_H2 / 8], bool src[CAM_W2][CAM_H2]);
//void imageConversionBack(Byte des[CAM_W * CAM_H / 8], bool src[CAM_W][CAM_H]);

void meanFilter(bool des[CAM_W][CAM_H], bool in[CAM_W][CAM_H]);
void centreFinder(vector<Coor>& cen, bool in[CAM_W][CAM_H]);

//void determinePts(vector<Coor>& pts, Coor& carH, Coor& carT, Coor& beacon);

float angleTracking(Coor carT, Coor carH);

//void assignDirection(Coor& newH, Coor& newT, Coor& carH, Coor& carT, float originalAngle);

//uint16_t calSize(Coor& target, bool inputImage[CAM_W][CAM_H]);
//bool compareSize(Coor& newH, Coor& newT, bool inputImage[CAM_W][CAM_H]);
//void determineCar(vector<Coor>& pts, Coor& carH, Coor& carT, Coor& beacon, bool inputImage[CAM_W][CAM_H]);

void sendSignal(const Coor& b, const Coor& cH, const Coor& cT);

//bool switchBeacon(Coor bn, Coor Cr);
uint32_t squaredDistance(const Coor& b, const Coor& c);

//void resolveDistortion(bool des[CAM_W][CAM_H], bool in[CAM_W][CAM_H]);

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

//Byte cameraBuffer[CAM_W * CAM_H / 8];
Byte cameraBuffer2[CAM_W2 * CAM_H2 / 8];
const Byte* cameraBuffer;

//get bit value from camerabuf using camera coordinate system
bool getBit(int i_x, int i_y){
	if (i_x<=0 || i_x>CAM_W-1 || i_y <= 0 || i_y > CAM_H-1) return -1;
	return cameraBuffer[i_y*CAM_W/8 + i_x/8] >> (7 - (i_x%8)) & 1;
}

//get bit using world coordinate system
bool getWorldBit(int w_x, int w_y){
	int i_x,i_y;
	i_x = transformMatrix[w_x][w_y][0];
	i_y = transformMatrix[w_x][w_y][1];
	return getBit(i_x,i_y);
}

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
//	Byte cameraBuffer[CAM_W * CAM_H / 8];
	bool boolImage[CAM_W][CAM_H];
	bool boolImageFiltered[CAM_W][CAM_H];
	bool boolImageWorld[CAM_W2][CAM_H2];
	vector<Coor> centre;

	Coor beacon;
	Coor carH;
	Coor carT;

/*	//initialize camera related arrays and vectors
	memset(cameraBuffer, 0, CAM_W * CAM_H / 8);
	for(int i = 0; i < CAM_W; i++)
		memset(boolImage + i, 0, CAM_H);
*/
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
		if( ( System::Time() - lastTime ) >= 150)
		{
			lastTime = System::Time();

			//update camera image
		//	memcpy(cameraBuffer, cameraP->LockBuffer(), CAM_W * CAM_H / 8);
			cameraBuffer = cameraP->LockBuffer();

			cameraP->UnlockBuffer();

			//formatting to boolean form
			for (int j = 0; j < CAM_H2; j++)
			{
				for (int i = 0; i < CAM_W2; i++)
				{
					boolImageWorld[i][j] = getWorldBit(i,j);
				}
			}
			//filtering
		//	meanFilter(boolImageFiltered, boolImage);

			//formatting to Byte from
	//		imageConversionBack(cameraBuffer, boolImage);
			imageConversionBack(cameraBuffer2, boolImageWorld);
/*
#ifdef ENABLE_LCD
			//print camera image easy version
			lcdP->SetRegion(Lcd::Rect(3, 2, CAM_W, CAM_H));
			//0 = white ************** in camera image
			//White = 0xFFFF = white in real
			lcdP->FillBits(0x0000, 0xFFFF, cameraBuffer, CAM_W * CAM_H);
#endif
*/

#ifdef	ENABLE_LCD
			//print camera image easy version
			lcdP->SetRegion(Lcd::Rect(3, 2, CAM_W2, CAM_H2));
			//0 = white ************** in camera image
			//White = 0xFFFF = white in real
			lcdP->FillBits(0x0000, 0xFFFF, cameraBuffer2, CAM_W2 * CAM_H2);
#endif


			//find centre of white regions
//			centreFinder(centre, boolImage); // boolImageFiltered

			/*
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
*//*#ifdef ENABLE_LCD
					lcdP->SetRegion(Lcd::Rect(3 + centre[a].x, 2 + centre[a].y, 2, 2));
					lcdP->FillColor(St7735r::kRed);
#endif*/
//				}
//			}

//			//update preCentre by using data in centre
		//	determinePts(centre, carH, carT, beacon);
//				determineCar(centre, carH, carT, beacon, boolImage);
			// //signal from the car to reset the data in the drone
			// if(startTheDroneProcess)
			// {
			// 	beacon = Coor();
			// 	car = Coor();
			// 	startTheDroneProcess = false;
			// }

			//send the two coordinates in preCentre to the Car with the first one the Car, the second one the Beacon
/*			sendSignal(beacon, carH, carT);


#ifdef ENABLE_LCD
			lcdP->SetRegion(Lcd::Rect(3 + beacon.x, 2 + beacon.y, 3, 3));
			lcdP->FillColor(St7735r::kPurple);
			lcdP->SetRegion(Lcd::Rect(3 + carH.x, 2 + carH.y, 3, 3));
			lcdP->FillColor(St7735r::kBlue);
			lcdP->SetRegion(Lcd::Rect(3 + carT.x, 2 + carT.y, 3, 3));
			lcdP->FillColor(St7735r::kCyan);

			lcdP->SetRegion(Lcd::Rect(0, 120, 128, 50));
			sprintf(buffer, "beacon V %d %d", beacon.x, beacon.y);
			writerP->WriteString(buffer);

			lcdP->SetRegion(Lcd::Rect(0, 140, 128, 50));
			sprintf(buffer, "carH B %d %d", carH.x, carH.y);
			writerP->WriteString(buffer);
#endif

			centre.clear();
*/
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
/*
void imageConversionBack(Byte des[CAM_W * CAM_H / 8], bool src[CAM_W][CAM_H])
{
	for(uint16_t i = 0; i < CAM_H * CAM_W / 8; i++)
		des[i] = 0;

	for(uint16_t i = 0; i < CAM_H; i++)
		for(uint16_t j = 0; j < CAM_W; j++)
			des[ (i * CAM_W + j) / 8 ] = ( src[j][i] << (7 - j % 8) ) | des[(i * CAM_W + j) / 8];
}
*/
void imageConversionBack(Byte des[CAM_W2 * CAM_H2 / 8], bool src[CAM_W2][CAM_H2])
{
	for(uint16_t i = 0; i < CAM_H2 * CAM_W2 / 8; i++)
		des[i] = 0;

	for(uint16_t i = 0; i < CAM_H2; i++)
		for(uint16_t j = 0; j < CAM_W2; j++)
			des[ (i * CAM_W2 + j) / 8 ] = ( src[j][i] << (7 - j % 8) ) | des[(i * CAM_W2 + j) / 8];
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

/*
void resolveDistortion(bool des[CAM_W][CAM_H], bool in[CAM_W][CAM_H])
{
	float kA = -0.000077;
	float kB = 0.000267;	//P
	float kC = 0;
	float kD = 1 - kA - kB - kC;
	float kP = 0.035;
	float kZ = 1;
//	uint8_t radius = CAM_H/2;
	uint8_t centreW = CAM_W/2;
	uint8_t centreH = CAM_H/2;
	double corrRadius = (sqrt(CAM_W * CAM_W + CAM_H * CAM_H))/kP;
	uint8_t newX = 0;
	uint8_t newY = 0;
	double dist = 0;
	double rate = 0;
	double kF = 0;
	uint8_t srcX = 0;
	uint8_t srcY = 0;
	for (int x = 0; x < CAM_W; x++)
	{
		newX = x-centreW;
		for (int y = 0; y < CAM_H; y++)
		{
			newY = y-centreH;
			dist = sqrt(newX * newX + newY * newY);
			rate = dist/corrRadius;
			if (rate == 0.0)
			{	kF = 1;	}
			else { kF = (atan(rate))/rate;	}
			srcX = (uint8_t)(centreW + kF * newX * kZ);
			srcY = (uint8_t)(centreH + kF * newY * kZ);
			des[x][y] = in[srcX][srcY];
		}
	}
/*
	for (int x = 0; x < CAM_W; x++)
	{
		for (int y = 0; y < CAM_H; y++)
		{
			double deltaX = (x - centreW) /radius;
			double deltaY = (y - centreH) /radius;
			double destR = Math::Sqrt2(deltaX * deltaX + deltaY * deltaY);
			double srcR = (kA * destR * destR * destR + kB * destR * destR + kC * destR + kD) * destR;
			double kF = destR/srcR;
			if (kF < 0)
			{	kF = -kF;	}
			uint8_t srcX = (uint8_t)(centreW + (deltaX * kF * radius));
			uint8_t srcY = (uint8_t)(centreH + (deltaY * kF * radius));
			if ((srcX > 0) && (srcY > 0) && (srcX < CAM_W) && (srcY < CAM_H))
			{
				des[x][y] = in[srcX][srcY];
			}
		}
	}
*/
//}


void centreFinder(vector<Coor>& cen, bool in[CAM_W][CAM_H])
{
	uint8_t distriX[CAM_W];
	uint8_t distriY[CAM_H];

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
						}
						uint64_t cenX = 0;
						uint64_t cenY = 0;
						uint32_t base = qForStore.size();
						for(uint16_t k = 0; k < base; k++)
						{
							Coor tempCoor = qForStore.front();
							qForStore.pop();
							cenX += tempCoor.x;
							cenY += tempCoor.y;
						}
						Coor tempCoor = Coor(cenX/base, cenY/base);
						cen.push_back(tempCoor);
					}//end of while loop for expanding the region
				}
			}
		}
	}//end of centrefinding for loop
}
/*
uint16_t calSize(Coor& target, bool inputImage[CAM_W][CAM_H])
{
	uint8_t xCount = 0;
	uint8_t yCount = 0;
	uint8_t xLimit = 40;
	uint8_t yLimit = 40;
	uint8_t countPixel = 1;
	bool countFlag = true;

	//	+ve x
	while (countFlag == true)
	{
		if (target.x + countPixel > (CAM_W - 1))
		{	countFlag = false;	}

		else
		{
			if (inputImage[target.x + countPixel][target.y] == 0)
			{
				if ( xCount < xLimit)
				{
					xCount++;
					countPixel++;
				}
			}
			else
			{	countFlag = false;	}
		}
	}
	countPixel = 1;
	countFlag = true;

	//	-ve x
	while (countFlag == true)
	{
		if (target.x - countPixel < 0)
		{	countFlag = false;	}

		else
		{
			if (inputImage[target.x - countPixel][target.y] == 0)
			{
				if ( xCount < xLimit)
				{
					xCount++;
					countPixel++;
				}
			}
			else
			{	countFlag = false;	}
		}
	}
	countPixel = 1;
	countFlag = true;

	//	+ve y
	while (countFlag == true)
	{
		if (target.y + countPixel > (CAM_H - 1))
		{	countFlag = false;	}

		else
		{
			if (inputImage[target.x][target.y + countPixel] == 0)
			{
				if ( yCount < yLimit)
				{
					yCount++;
					countPixel++;
				}
			}
			else
			{	countFlag = false;	}
		}
	}
	countPixel = 1;
	countFlag = true;

	//	-ve y
	while (countFlag == true)
	{
		if (target.y + countPixel < 0)
		{	countFlag = false;	}

		else
		{
			if (inputImage[target.x][target.y - countPixel] == 0)
			{
				if ( yCount < yLimit)
				{
					yCount++;
					countPixel++;
				}
			}
			else
			{	countFlag = false;	}
		}
	}
	return 1 + xCount + yCount;
}

bool compareSize(Coor& newH, Coor& newT, bool inputImage[CAM_W][CAM_H])
{
	// false -> need to swap H and T
	uint16_t size1 = calSize(newH, inputImage);
	uint16_t size2 = calSize(newT, inputImage);
	if (size1 > size2)
	{	return true;	}
	else {	return false;	}
}

void determineCar(vector<Coor>& pts, Coor& carH, Coor& carT, Coor& beacon, bool inputImage[CAM_W][CAM_H])
{
	uint8_t cCount = pts.size();
	if (cCount == 3)
	{
		uint16_t d1 = squaredDistance(pts[0], pts[1]);
		uint16_t d2 = squaredDistance(pts[0], pts[2]);
		uint16_t d3 = squaredDistance(pts[1], pts[2]);

		uint16_t shortestD = d1;			// pt 0/1 are car

		if(shortestD > d2)
		{	shortestD = d2;	}				// pt 0/2 are car
		if(shortestD > d3)
		{	shortestD = d3;	}				// pt 1/2 are car

		if (shortestD == d1)
		{
			if (compareSize(pts[0], pts[1], inputImage) == true )
			{
				carH = pts[0];
				carT = pts[1];
			}
			else
			{
				carH = pts[1];
				carT = pts[0];
			}
			beacon = pts[2];
		}

		else if (shortestD == d2)
		{
			if (compareSize(pts[0], pts[2], inputImage) == true )
			{
				carH = pts[0];
				carT = pts[2];
			}
			else
			{
				carH = pts[2];
				carT = pts[0];
			}
			beacon = pts[1];
		}
		else
		{
			if (compareSize(pts[1], pts[2], inputImage) == true )
			{
				carH = pts[1];
				carT = pts[2];
			}
			else
			{
				carH = pts[2];
				carT = pts[1];
			}
			beacon = pts[0];
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
*/
float angleTracking(Coor carT, Coor carH)
{
	// -180 to 180 deg, 180 for LHS/ -180 for RHS
	int deltaX = carH.x - carT.x;
	int deltaY = carH.y - carT.y;
	if (deltaX != 0)
	{
		// The degree that the image has rotated
		// rad -> deg
		float angle = 180 * atan(abs(deltaY)/abs(deltaX))/ M_PI;
		if (deltaX > 0)
		{
			if (deltaY > 0)
			{	return (-90 - angle);	}
			else
			{	return (-90 + angle);	}
		}
		else
		{
			if (deltaY > 0)
			{	return (90 + angle);	}
			else
			{	return (90 - angle);	}
		}
	}
	else
	{
		if (deltaY < 0)
		{	return 0;	}
		else if(deltaY > 0)
		{	return 180;	}
	}
}

void sendSignal(const Coor& b, const Coor& cH, const Coor& cT)
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
	if(cH == Coor())
	{
		tempMessage += "-01,";
		tempMessage += "-01,";
	}else
	{
		if(cH.x <= 9)
		{
			sprintf(buffer, "00%d,", cH.x);
		}else if(cH.x <= 99)
		{
			sprintf(buffer, "0%d,", cH.x);
		}else
		{
			sprintf(buffer, "%d,", cH.x);
		}

		tempMessage += buffer;

		if(cH.y <= 9)
		{
			sprintf(buffer, "00%d,", cH.y);
		}else if(cH.y <= 99)
		{
			sprintf(buffer, "0%d,", cH.y);
		}else
		{
			sprintf(buffer, "%d,", cH.y);
		}

		tempMessage += buffer;
	}
	if(cT == Coor())
		{
			tempMessage += "-01,";
			tempMessage += "-01,";
		}else
		{
			if(cT.x <= 9)
			{
				sprintf(buffer, "00%d,", cT.x);
			}else if(cT.x <= 99)
			{
				sprintf(buffer, "0%d,", cT.x);
			}else
			{
				sprintf(buffer, "%d,", cT.x);
			}

			tempMessage += buffer;

			if(cT.y <= 9)
			{
				sprintf(buffer, "00%d,", cT.y);
			}else if(cT.y <= 99)
			{
				sprintf(buffer, "0%d,", cT.y);
			}else
			{
				sprintf(buffer, "%d,", cT.y);
			}

			tempMessage += buffer;
		}
	tempMessage += 'e';
	bluetoothP->SendStr(tempMessage);
}

uint32_t squaredDistance(const Coor& b, const Coor& c)
{
	int32_t xCom = b.x - c.x;
	int32_t yCom = b.y - c.y;
	return ( xCom * xCom ) + ( yCom * yCom );
}

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
