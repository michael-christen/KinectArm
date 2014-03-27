#ifndef BODY_H
#define BODY_H

typedef struct Body body_t;
typedef struct Joint joint_t;

struct Joint{
	int x, y, z;
	int screen_x, screen_y;
};

struct Body{
	joint_t head;
	joint_t left_shoulder;
	joint_t right_shoulder;
	joint_t left_elbow;
	joint_t right_elbow;
	joint_t left_wrist;
	joint_t right_wrist;
	joint_t left_finger;
	joint_t right_finger;
	int use_right; //1 if right, 0 if left
};

void getServoAnglesFromBody(body_t* body, double servoAngles[]);

#endif
