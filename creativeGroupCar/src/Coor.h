
#ifndef COOR_H
#define COOR_H

#include <cstdint>

struct Coor
{
	int16_t x;
	int16_t y;

	Coor():x(-1), y(-1)
	{
	}

	Coor(const int16_t& X, const int16_t& Y):x(X), y(Y)
	{
	}

	Coor(const Coor& c):x(c.x), y(c.y)
	{
	}

	bool operator==(const Coor& r) const
	{
		if(x == r.x && y == r.y)
			return true;
		else
			return false;
	}

	bool operator!=(const Coor& r) const
	{
		return !this->operator==(r);
	}

	void reset()
	{
		x = -1;
		y = -1;
	}

	Coor& operator=(const Coor& r)
	{
		x = r.x;
		y = r.y;
		return *this;
	}

};

#endif
