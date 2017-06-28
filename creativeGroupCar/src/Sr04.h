/*
 * Sr04.h
 *
 *  Created on: 28 Jun 2017
 *      Author: ChungWa
 */

#ifndef SR04_H
#define SR04_H

#include <vector>
#include <libsc/system.h>
#include <libbase/k60/gpio.h>
#include <libbase/k60/pin.h>

using namespace libbase;
using namespace k60;
using namespace libsc;

//ultrasonic sensor
class Sr04
{
	uint32_t time;
	float range;
public:
	struct Config
	{
		Pin::Name echo;
		Pin::Name trigger;
	};
	explicit Sr04(const Config& config);

	static std::vector<Sr04*> ultraPVector;

	static void shoot();

	float getRange()
	{
		range *= 0.02125;
		return range > 4.0? 4.0 : range;
	}

	Gpi echo;
	Gpo trigger;
};



#endif /* SR04_H */
