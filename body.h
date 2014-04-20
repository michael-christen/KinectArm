#ifndef BODY_H
#define BODY_H

#include "joint.h"

#define NUM_JOINTS 10

class Body {
	public:
		joint_t getJoint(Joints jointid);
		void setJoint(Joints jointid, joint_t joint);
	private:
		joint_t joints[NUM_JOINTS];
};

#endif
