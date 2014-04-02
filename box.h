#ifndef BOX_H
#define BOX_H

class Box {
	private:
		double x1, y1, z1, x2, y2, z2;
		double width, length, depth; // w - dx, l - dy, d - dz
	public:
		Box(double x1, double y1, double z1, double x2, double y2, double z2);
		bool intersect(Box *b1, Box *b2);
};

#endif