/* -.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.

 * File Name : joint.cpp

 * Purpose :

 * Creation Date : 17-04-2014

 * Last Modified : Fri 18 Apr 2014 12:26:49 AM EDT

 * Created By : Michael Christen

 _._._._._._._._._._._._._._._._._._._._._.*/
#include "joint.h"
joint_t getReal(Image<uint16_t> & depth, int id) {
	int x      = depth.getX(id);
	int y      = depth.getY(id);
	uint16_t d = depth.get(id);
	joint_t temp;
	temp.screen_x = x;
	temp.screen_y = y;
	temp.x    = GetRealWorldXFromDepth(d,x);
	temp.y    = GetRealWorldXFromDepth(d,y);
	temp.z    = d;
	return temp;
}

