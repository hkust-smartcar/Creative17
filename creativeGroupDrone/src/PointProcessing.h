
#include <cstdint>
#include <vector>
#include "Coor.h"
//#include <cassert>
//#include <cstring>
//#include <queue>

using namespace std;

#define DISTANCE_CHECK 30

void determinePts(vector<Coor>& pt, Coor& beacon, Coor& car);
void multiplePts(vector<Coor>& pts, Coor& Beacon, Coor& Car, int carOnly);
bool switchBeacon(Coor bn, Coor Cr);
uint32_t squaredDistance(const Coor& b, const Coor& c);
