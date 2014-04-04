#ifndef CONFIG_SPACE_H
#define CONFIG_SPACE_H

#include "bounding_box.h"
#include <vector>

class ConfigSpace {
	public:
		void addBoundingBox(BoundingBox box);
	private:
		std::vector<BoundingBox> boxes;
};

#endif
