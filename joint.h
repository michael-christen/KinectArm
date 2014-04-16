#ifndef JOINT_H
#define JOINT_H

enum Joints { HEAD, RSHOULDER, RELBOW, RWRIST, LSHOULDER, LELBOW, LWRIST };

struct joint_t {
	double x, y, z;
	double screen_x, screen_y;
};

#endif