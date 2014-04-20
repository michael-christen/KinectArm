#include "body.h"
#include "eecs467_util.h"
#include <sys/time.h>
#include <cmath>

joint_t Body::getJoint(Joints jointid) {
	return this->joints[jointid];
}

void Body::setJoint(Joints jointid, joint_t joint) {
	this->joints[(int)jointid] = joint;
}

void Body::setWristRotation(double rotation) {
	this->wrist_rotation = rotation;
}

double Body::getWristRotation() {
	return this->wrist_rotation;
}

void Body::setJointDistThresh(double jointDistThresh) {
	this->jointDistThresh = jointDistThresh;
}

bool Body::getValidJointPosition(Joints jointid, joint_t joint) {
	double dist = pow(joint.x - this->joints[jointid].x, 2);
	dist += pow(joint.y - this->joints[jointid].y, 2);
	dist += pow(joint.z - this->joints[jointid].z, 2);
	dist = sqrt(dist);

	if (dist < jointDistThresh) {
		jointValid[jointid] = true;
		return true;
	} else {
		double now = utime_now();
		if (jointValid[jointid]) {
			jointValid[jointid] = false;
			jointInvalidTS[jointid] = now;
			//printf("Invalid at - %f\n", now);	
			return false;		
		} else {
			double timeChange = now - jointInvalidTS[jointid];
			//printf("%d Time change - %f, %f\n", jointid, timeChange, this->invalidTimeout);
			if (timeChange > this->invalidTimeout) {
				jointValid[jointid] = true;
				return true;
			} else {
				return false;
			}
		}
	}
}

void Body::setInvalidJointTimeout(double timeout) {
	this->invalidTimeout = timeout;
}