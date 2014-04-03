/* -.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.

      * File Name : Gradient.cpp

         * Purpose :

	    * Creation Date : 03-04-2014

	       * Last Modified : Thu 03 Apr 2014 02:38:09 PM EDT

	          * Created By : Michael Christen

		     _._._._._._._._._._._._._._._._._._._._._.*/
#include "Gradient.h"

Gradient::Gradient()
	:X(0.0), Y(0.0) {}

void Gradient::x(double val) {
	X = val;
}

void Gradient::y(double val) {
	Y = val;
}

double Gradient::y() {
	return Y;
}

double Gradient::x() {
	return X;
}
double Gradient::mag() {
	return sqrt(pow(X,2)+pow(Y,2));
}
double Gradient::angle() {
	return atan2(X,Y);
}
void Gradient::print() {
	printf("x:%f,y:%f, |gr|:%f, <gr:%f\n",
			X,Y,mag(),angle()
	);
}


