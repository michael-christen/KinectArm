#include "body.h"

joint_t Body::getJoint(Joints jointid) {
	return this->joints[jointid];
}

void Body::setJoint(Joints jointid, joint_t joint) {
	this->joints[(int)jointid] = joint;
}
