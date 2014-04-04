#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

#include "../common/matd.h"

class BoundingBox {
	private:
		double hW, hH, hD;
		matd_t *pos, *ux, *uy, *uz;
	public:
		BoundingBox();
		~BoundingBox();
		void setPosition(double x, double y, double z);
		void setDimensions(double w, double h, double d);
		void setRotation(double rX, double rY, double rZ);
		bool intersect(BoundingBox *b);
};

#endif