#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

#include "../common/matd.h"

class BoundingBox {
	private:
		double hW, hH, hD, w, h, d;
		matd_t *pos, *ux, *uy, *uz;
	public:
		BoundingBox();
		~BoundingBox();
		void setPosition(double x, double y, double z);
		void setDimensions(double w, double h, double d);
		void rotateUnitVectors(double tx, double ty, double tz);
		bool intersect(BoundingBox *b);
		double getX();
		double getY();
		double getZ();
		double getW();
		double getH();
		double getD();
		double getHw();
		double getHh();
		double getHd();
};

#endif
