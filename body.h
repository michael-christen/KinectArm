#ifndef BODY_H
#define BODY_H

#include "joint.h"

#define NUM_JOINTS 10

class Body {
	public:
		joint_t getJoint(Joints jointid);
		void setJoint(Joints jointid, joint_t joint);
		void setJointDistThresh(double jointDistThresh);
		bool getValidJointPosition(Joints jointid, joint_t joint);
		void setInvalidJointTimeout(double timeout);
	private:
		joint_t joints[NUM_JOINTS];
		double jointDistThresh;
		bool jointValid[NUM_JOINTS];
		double jointInvalidTS[NUM_JOINTS];
		double invalidTimeout;
};

#endif
