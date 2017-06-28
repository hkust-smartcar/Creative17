
#include "Sr04.h"
namespace
{

	Gpi::Config getGpiConfig(const Sr04::Config& c, uint32_t * const t, float * const r)
	{
		Gpi::Config config;
		config.pin = c.echo;
		config.interrupt = Pin::Config::Interrupt::kBoth;
		config.isr = [=] (Gpi* gpi)
		{
			if (gpi->Get())
			{ *t =  System::TimeIn125us(); }
			else
			{ *r = System::TimeIn125us() - *t; }
		};
		return config;
	}

	Gpo::Config getGpoConfig(const Sr04::Config& c)
	{
		Gpo::Config config;
		config.pin = c.trigger;
		config.is_high = false;
		return config;
	}

}

std::vector<Sr04*> Sr04::ultraPVector;

Sr04::Sr04(const Config& config)
:time(0),
 range(0.0),
 echo(getGpiConfig(config, &time, &range)),
 trigger(getGpoConfig(config))
{
	ultraPVector.push_back(this);
}

void Sr04::shoot()
{
//	for(std::vector<Sr04*>::iterator it = ultraPVector.begin(); it != ultraPVector.end(); it++)
//		((*it)->trigger).Set();

	for(uint8_t i = 0; i < ultraPVector.size(); i++)
		ultraPVector[i]->trigger.Set();
	uint32_t startTimeForUltra = System::TimeIn125us();
	while(System::TimeIn125us() < startTimeForUltra + 2){;}
//	for(std::vector<Sr04*>::iterator it = ultraPVector.begin(); it != ultraPVector.end(); it++)
//		((*it)->trigger).Reset();

	for(uint8_t i = 0; i < ultraPVector.size(); i++)
		ultraPVector[i]->trigger.Reset();
}
