#include "body.h"
#include "../common/matd.h"
#include "../common/math_util.h"
#include "math.h"
#include "skeleton_joint_list_t.h"
#include "skeleton_joint_t.h"

Body::Body(const skeleton_joint_list_t *msg) {
	joint_t *joint;
	for (int i = 0; i < msg->len; i++) {
		switch (i) {
			case 0: joint = &(this->head); break;
			case 1: joint = &(this->right_shoulder); break;
			case 2: joint = &(this->right_elbow); break;
			case 3: joint = &(this->right_wrist); break;
			case 4: joint = &(this->left_shoulder); break;
			case 5: joint = &(this->left_elbow); break;
			default: joint = &(this->left_wrist); break;
		}

		joint->x = (double) msg->joints[i].x;
		joint->y = (double) msg->joints[i].y;
		joint->z = (double) msg->joints[i].z;
		joint->screen_x = (double) msg->joints[i].screen_x;
		joint->screen_y = (double) msg->joints[i].screen_y;
	}
}

void Body::getServoAngles(double servoAngles[], bool right_side){
	matd_t* floor_shoulder;
	matd_t* shoulder_elbow;
	matd_t* shoulder_elbow0;
	matd_t* shoulder_elbow1;
	matd_t* elbow_wrist;
	joint_t shoulder, elbow, wrist;

	if(right_side){
		//use right side of body
		shoulder = this->right_shoulder;
		elbow = this->right_elbow;
		wrist = this->right_wrist;
	}else{
		//use left side of body
		shoulder = this->left_shoulder;
		elbow = this->left_elbow;
		wrist = this->left_wrist;
	}

	double floor_shoulder_data[3] = {
		0, 0, shoulder.z};

	double shoulder_elbow_data[3] = {
		elbow.x - shoulder.x,
		elbow.y - shoulder.y,
		elbow.z - shoulder.z};

	double shoulder_elbow_data0[3] = {
		0,
		elbow.y - shoulder.y,
		elbow.z - shoulder.z}; 

	double shoulder_elbow_data1[3] = {
		elbow.x - shoulder.x,
		0,
		elbow.z - shoulder.z};
		
	double elbow_wrist_data[3] = {
		wrist.x - elbow.x,
		wrist.y - elbow.y,
		wrist.z - elbow.z};

	floor_shoulder = matd_create_data(3, 1, floor_shoulder_data);
	shoulder_elbow = matd_create_data(3, 1, shoulder_elbow_data);
	shoulder_elbow0 = matd_create_data(3, 1, shoulder_elbow_data0);
	shoulder_elbow1 = matd_create_data(3, 1, shoulder_elbow_data1);
	elbow_wrist = matd_create_data(3, 1, elbow_wrist_data);

	double magfs = matd_vec_mag(floor_shoulder);
	double magse = matd_vec_mag(shoulder_elbow);
	double magse0 = matd_vec_mag(shoulder_elbow0);
	double magse1 = matd_vec_mag(shoulder_elbow1);
	double magew = matd_vec_mag(elbow_wrist);
	
	double shoulderValue0 = matd_vec_dot_product(floor_shoulder, shoulder_elbow0) / (magfs * magse0);
	double shoulderValue1 = matd_vec_dot_product(floor_shoulder, shoulder_elbow1) / (magfs * magse1);
	double elbowValue = matd_vec_dot_product(shoulder_elbow, elbow_wrist) / (magse * magew);

	double wristx = wrist.x - elbow.x;
	double wristy = wrist.y - elbow.y;
	double posElbowAngle = atan2(wristy, wristx);

	printf("s0 - %f, s1 - %f, e - %f, pe - %f\n", shoulderValue0, shoulderValue1, elbowValue, posElbowAngle);

	double shoulderAngle0 = -sgn(shoulderValue0)*acos(shoulderValue0);
	double shoulderAngle1 = sgn(elbowValue)*(acos(shoulderValue1) - M_PI/2);
	double elbowAngle = acos(elbowValue);

	servoAngles[0] = shoulderAngle0;
	servoAngles[1] = shoulderAngle1;
	servoAngles[2] = elbowAngle;

	matd_destroy(floor_shoulder);
	matd_destroy(shoulder_elbow);
	matd_destroy(shoulder_elbow0);
	matd_destroy(shoulder_elbow1);
	matd_destroy(elbow_wrist);
}
