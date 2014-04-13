#ifndef BODY_H
#define BODY_H

#include "skeleton_joint_list_t.h"
#include "data_smoother.h"
#include "vx/vx.h"
#include "joint.h"

#define NUM_JOINTS 7

class Body {
	public:
		DataSmoother *ds;
		Body();
		~Body();
		void processMsg(const skeleton_joint_list_t *msg);
		void getServoAngles(double servoAngles[], bool right_side);
		void draw(vx_buffer_t *buf, const float bone_color[], const float joint_color[]);

	private:
		joint_t joints[NUM_JOINTS];
};

#endif
