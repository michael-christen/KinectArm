#ifndef REXARM_H
#define REXARM_H

#define NUM_SERVOS 6

#include "vx/vx.h"
#include <pthread.h>

class RexArm {
	public:
		RexArm();
		void setTargetAngles(double angles[]);
		void setCurAngles(double angles[]);
		void getTargetAngles(double arr[]);
		void getCurAngles(double arr[]);
		void drawCurState(vx_buffer_t *buf, const float color[]);
		void drawTargetState(vx_buffer_t *buf, const float color[]);
	private:
		pthread_mutex_t curAnglesMutex, targetAnglesMutex;
		static const int numServos;
		static const double segmentLength[5];
		static const double segmentWidth[5];
		static const double segmentDepth[5];
		double curAngles[NUM_SERVOS];
	    double targetAngles[NUM_SERVOS];
	    void drawState(vx_buffer_t *buf, const float color[], double angles[]);
};

#endif
