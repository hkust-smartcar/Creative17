#include <cassert>
#include <cmath>
#include <cstdint>

#include <array>
#include <vector>

#include "libbase/log.h"
#include "libbase/helper.h"
#include LIBBASE_H(i2c_master)
#include LIBBASE_H(soft_i2c_master)

#include "libsc/config.h"
#include "Qmc5883.h"
#include "libsc/system.h"
#include "libutil/misc.h"

#define ABS(v) ((v < 0)? -v : v)

using namespace LIBBASE_NS;
using namespace std;

namespace libsc
{

#ifdef LIBSC_USE_QMC5883

namespace
{

QMC5883::I2cMaster::Config GetI2cConfig()
{
	QMC5883::I2cMaster::Config config;
	config.scl_pin = LIBSC_QMC5883_SCL;
	config.sda_pin = LIBSC_QMC5883_SDA;
	config.baud_rate_khz = 400;
	config.scl_low_timeout = 1000;
#if !LIBSC_USE_SOFT_QMC5883
	config.min_scl_start_hold_time_ns = 600;
	config.min_scl_stop_hold_time_ns = 600;
#endif
	return config;
}

}

Qmc5883::Qmc5883(const Config &config)
:m_mag{},
m_i2c(0)
{
	if(!config.i2c_master_ptr){
		m_i2c = new I2cMaster(GetI2cConfig());
	}else{
		m_i2c = config.i2c_master_ptr;
	}

	softReset();

	//define Set/Reset period
	assert(m_i2c->SendByte(QMC5883_ADDR, 0x0B, 0x01));
	System::DelayUs(1);

	//set mode

	/*
	Define
	OSR = 512
	Full Scale Range = 8G(Gauss)
	ODR = 200HZ
	set continuous measurement mode
	*/
	assert(m_i2c->SendByte(QMC5883_ADDR, 0x09, Mode_Continuous | ODR_200Hz | RNG_8G | OSR_512));
	System::DelayUs(1);

	//make sure the min and max are in the right range by giving them initial value
	System::DelayMs(8);
	Update();
	for (int i = 0; i < 3; ++i)
	{
		m_mag_min[i] = m_mag_max[i] = m_mag[i];
	}
}

bool Qmc5883::Update()
{
	const vector<Byte> &data = m_i2c->GetBytes(QMC5883_ADDR, 0x00, 6);
	if (data.empty())
		return false;

	for (size_t i = 0; i < data.size(); i += 2)
	{
		m_mag[i / 2] =  data[i];
		m_mag[i / 2] |=  (data[i + 1] << 8);
	}
	return true;
}

bool UpdateWithCalibration()
{
	Update();
	for (int i = 0; i < 3; ++i)
	{
		if(m_mag[i] > m_mag_max[i])
			m_mag_max[i] = m_mag[i];
		else if (m_mag[i] < m_mag_min[i])
			m_mag_min[i] = m_mag[i];
	}
}

void Qmc5883::softReset()
{
	assert(m_i2c->SendByte(QMC5883_ADDR, 0x0A, 0x80));
	System::DelayUs(1);
}

std::array<int32_t, 3> Qmc5883::GetNormalizedMag() const
{
	std::array<int32_t, 3> tempArray;
	for (int i = 0; i < 3; ++i)
	{
		uint32_t maxMagnitude = (m_mag_max[i] - m_mag_min[i]) / 2;
		uint32_t shitedCentre = (m_mag_max[i] + m_mag_min[i]) / 2;

		//the output will be from -10000 to 10000
		tempArray[i] = 10000 * (m_mag - shitedCentre) / maxMagnitude;
	}

	return tempArray;
}

#else
Qmc5883::Qmc5883(const Config&)
		: m_i2c(nullptr)
{
	LOG_DL("Configured not to use Qmc5883");
}
bool Qmc5883::Update() { return false; }

#endif /* LIBSC_USE_Qmc5883 */

}