
#include <cstdint>
#include <vector>
#include "Coor.h"
//#include <cassert>
//#include <cstring>
//#include <queue>
#include "PointProcessing.h"

using namespace std;

#define DISTANCE_CHECK 30

void determinePts(vector<Coor>& pt, Coor& beacon, Coor& car, uint8_t prevCentreCount, bool runFlag)
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
		//	beacon_old = beacon = pt[0];
			runFlag = 1;
		}
			//		else if (centreCount > 1)
			//		{	centreStorage = centre;	}
	}
	// 2nd Run/ beacon is already found
	else if (runFlag == 1)
	{
		if (centreNo > 0)
			// Calculate beacon & new pt dist
		{	d1 = (beacon.x - pt[0].x) * (beacon.x - pt[0].x) + (beacon.y - pt[0].y) * (beacon.y - pt[0].y);	}

		//	Last time only 1 centre is found
		if (prevCentreCount == 1)
		{
			//	This time also only 1 centre is found
			if (centreNo == 1)
			{
					if (switchBeacon(beacon, car) == false)
						// Update beacon Coor
					{	beacon = pt[0];	}
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
						// Update beacon Coor
					{	beacon = pt[0];	}
			}
			// More than 1 pts -> Update both beacon and car
			else if (centreNo > 1)
			{	multiplePts(pt, beacon, car, 0);	}
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

uint32_t squaredDistance(const Coor& b, const Coor& c)
{
	int32_t xCom = b.x - c.x;
	int32_t yCom = b.y - c.y;
	return ( xCom * xCom ) + ( yCom * yCom );
}
