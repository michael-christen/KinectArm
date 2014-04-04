#ifndef CONFIG_SPACE_H
#define CONFIG_SPACE_H

#include "bounding_box.h"
#include <vector>
#include "vx/vx.h"

class ConfigSpace {
	public:
		void addBoundingBox(BoundingBox &box);
		void draw(vx_buffer *buf, const float color[]);
	private:
		std::vector<BoundingBox> boxes;
};

#endif
