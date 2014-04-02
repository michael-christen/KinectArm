#include "../common/matd.h"
#include "math.h"
#include "box.h"

Box::Box(double x1, double y1, double z1, double x2, double y2, double z2) {
	this->x1 = x1;
	this->y1 = y1;
	this->z1 = z1;
	this->x2 = x2;
	this->y2 = y2;
	this->z2 = z2;
}
