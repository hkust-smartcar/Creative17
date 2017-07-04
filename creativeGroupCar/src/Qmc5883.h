/*
 * Qmc5883.h
 *
 *  Created on: 3 Jul 2017
 *      Author: ChungWa
 */

#ifndef QMC5883_H
#define QMC5883_H







#include <cstdint>
#include <array>

#include "libbase/helper.h"
#include "libbase/misc_types.h"
#include LIBBASE_H(i2c_master)
#include LIBBASE_H(soft_i2c_master)

#include "libsc/config.h"

namespace libsc
{

#define QMC5883_ADDR 0x0D


//REG CONTROL

//0x09

#define Mode_Standby    0b00000000
#define Mode_Continuous 0b00000001

#define ODR_10Hz        0b00000000
#define ODR_50Hz        0b00000100
#define ODR_100Hz       0b00001000
#define ODR_200Hz       0b00001100

#define RNG_2G          0b00000000
#define RNG_8G          0b00010000

#define OSR_512         0b00000000
#define OSR_256         0b01000000
#define OSR_128         0b10000000
#define OSR_64          0b11000000

class Qmc5883
{
public:
#if LIBSC_USE_SOFT_QMC5883
	typedef LIBBASE_MODULE(SoftI2cMaster) I2cMaster;
#else
	typedef LIBBASE_MODULE(I2cMaster) I2cMaster;
#endif // LIBSC_USE_SOFT_QMC5883

	struct Config
	{
		I2cMaster* i2c_master_ptr = NULL;
	};

	explicit Qmc5883(const Config &config);

	bool Update();

	bool UpdateWithCalibration();

	void SoftReset();

	const std::array<int16_t, 3>& GetMag() const
	{
		return m_mag;
	}

	const std::array<int16_t, 3>& GetMagMin() const
	{
		return m_mag_min;
	}

	const std::array<int16_t, 3>& GetMagMax() const
	{
		return m_mag_max;
	}

	std::array<int32_t, 3> GetNormalizedMag() const;

	I2cMaster* GetI2cMaster(){
		return m_i2c;
	}

private:

	I2cMaster* m_i2c;
	std::array<int16_t, 3> m_mag;
	std::array<int16_t, 3> m_mag_min;
	std::array<int16_t, 3> m_mag_max;
};

}













#endif /* QMC5883_H */
