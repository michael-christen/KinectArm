#ifndef BODY_H
#define BODY_H

#include "skeleton_joint_list_t.h"

struct joint_t {
	double x, y, z;
	double screen_x, screen_y;
};

class Body {
	public:
		Body(const skeleton_joint_list_t *msg);
		void getServoAngles(double servoAngles[], bool right_side);

	private:
		joint_t head;
		joint_t left_shoulder;
		joint_t right_shoulder;
		joint_t left_elbow;
		joint_t right_elbow;
		joint_t left_wrist;
		joint_t right_wrist;
		joint_t left_finger;
		joint_t right_finger;
};

#endif
