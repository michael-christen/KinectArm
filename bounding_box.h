#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

#include "../common/matd.h"

#include "vx/vx.h"

class BoundingBox {
	private:
		double hW, hH, hD, w, h, d, tx, ty, tz;
		matd_t *pos, *ux, *uy, *uz;
	public:
		BoundingBox();
		~BoundingBox();
		void reset();
		void setPosition(double x, double y, double z);
		void setPosition(matd_t *pos);
		void setRotations(double tx, double ty, double tz);
		void addRotations(double tx, double ty, double tz);
		void setDimensions(double w, double h, double d);
		void setUnitVectors(matd_t *ux, matd_t *uy, matd_t *uz);
		bool intersect(BoundingBox *b);
		void draw(vx_buffer *buf, const float color[]);
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
