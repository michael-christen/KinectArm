#include "../common/matd.h"
#include "math.h"
#include "box.h"

/*
	x1 <= x2, y1 <= y2, z1 <= z2
*/


Box::Box(double x1, double y1, double z1, double x2, double y2, double z2) {
	if (x1 <= x2) {
		this->x1 = x1;
		this->x2 = x2;
	} else {
		this->x1 = x2;
		this->x2 = x1;
	}

	if (y1 <= y2) {
		this->y1 = y1;
		this->y2 = y2;
	} else {
		this->y1 = y2;
		this->y2 = y1;
	}

	if (z1 <= z2) {
		this->z1 = z1;
		this->z2 = z2;
	} else {
		this->z1 = z2;
		this->z2 = z1;
	}

	this->width = x2 - x1;
	this->length = y2 - y1;
	this->depth = z2 - z1;
}

bool Box::intersect(Box *b1, Box *b2) {
	//if (b2->x1 >= b1->x1 - b2-> )
	return false;
}
