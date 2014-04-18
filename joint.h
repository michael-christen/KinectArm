#ifndef JOINT_H
#define JOINT_H
#include "Image.h"

enum Joints { 
	HEAD, RSHOULDER, RELBOW, RWRIST, 
	LSHOULDER, LELBOW, LWRIST,
	MIDPOINT, LFOOT, RFOOT
};

struct joint_t {
	double x, y, z;
	int screen_x, screen_y;
};

joint_t getReal(Image<uint16_t> & depth, int id); 

#endif
