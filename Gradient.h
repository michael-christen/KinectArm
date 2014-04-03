#ifndef __GRADIENT__H__
#define __GRADIENT__H__
#include <cmath>

class Gradient {
	public:
		Gradient();
		void x(double);
		double x();
		void y(double);
		double y();
		double mag();
		double angle();
	private:
		double X;
		double Y;
};


#endif
