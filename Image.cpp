/* -.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.

      * File Name : Image.cpp

         * Purpose :

	    * Creation Date : 11-04-2014

	       * Last Modified : Fri 11 Apr 2014 02:37:51 PM EDT

	          * Created By : Michael Christen

		     _._._._._._._._._._._._._._._._._._._._._.*/
#include "Image.h"
double GetRealWorldXFromDepth(uint16_t depth, int x) {
    const double xFieldOfView = 57.0f;
    const double halfXFieldOfView = xFieldOfView / 2;
    const int imWidth = 640;
    const double halfWidth = imWidth / 2.f;
    const double xFocal = halfWidth / tan(halfXFieldOfView);
    
    return (static_cast<double>(x) - halfWidth) * (static_cast<double>(depth) - 10) * 0.0021;
}

double GetRealWorldYFromDepth(uint16_t depth, int y) {
    const double yFieldOfView = 43.0f;
    const double halfYFieldOfView = yFieldOfView / 2;
    const int imHeight = 480;
    const double halfHeight = imHeight / 2.f;
    const double yFocal = halfHeight / tan(halfYFieldOfView);
    
    return (static_cast<double>(y) - halfHeight) * (static_cast<double>(depth) - 10) * 0.0021;
    
} 


