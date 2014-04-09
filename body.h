#ifndef BODY_H
#define BODY_H

#include "skeleton_joint_list_t.h"
#include "data_smoother.h"

#define NUM_JOINTS 7

enum Joints { HEAD, RSHOULDER, RELBOW, RWRIST, LSHOULDER, LELBOW, LWRIST };

struct joint_t {
	double x, y, z;
	double screen_x, screen_y;

};

class Body {
	public:
		DataSmoother *ds;
		Body();
		~Body();
		void processMsg(const skeleton_joint_list_t *msg);
		void getServoAngles(double servoAngles[], bool right_side);

	private:
		joint_t joints[NUM_JOINTS];
};

#endif
