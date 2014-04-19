#ifndef REXARM_H
#define REXARM_H

#define NUM_SERVOS 6
#define NUM_SEGMENTS 5

#include "vx/vx.h"
#include <pthread.h>
#include "config_space.h"
#include "bounding_box.h"
#include "fsm_state.h"

class RexArm {
	public:
		RexArm();
		void setTargetAngles(double newAngles[], ConfigSpace &cfs);
		void setCurAngles(double angles[]);
		void setTargetSpeed(double speed);
		void getTargetAngles(double arr[]);
		void getCurAngles(double arr[]);
		void getTargetSpeed(double& speed);
		void drawCurState(vx_buffer_t *buf, const float color[], FSM_state_t state);
		void drawTargetState(vx_buffer_t *buf, const float color[], FSM_state_t state);

	private:
		pthread_mutex_t curAnglesMutex, targetAnglesMutex, targetSpeedMutex;
		static const int numServos;
		static const double segmentWidth[4];
		static const double segmentHeight[4];
		static const double segmentDepth[4];
		static const double segmentZOffset;
		BoundingBox segments[NUM_SEGMENTS];
		double curAngles[NUM_SERVOS];
		double prevAngles[NUM_SERVOS];
		double prevTrendFactors[NUM_SERVOS];
	    double targetAngles[NUM_SERVOS];
		double targetSpeed;
	    double dsf, tsf;
	    ConfigSpace *cfs;
	    void drawState(vx_buffer_t *buf, const float color[], double angles[], FSM_state_t state);
	    void setBoundingBoxes(BoundingBox boxes[]);
};

#endif
