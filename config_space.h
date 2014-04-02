#ifndef CONFIG_SPACE_H
#define CONFIG_SPACE_H

#include "box.h"
#include <vector>

class ConfigSpace {
	public:
		void addBox();
	private:
		std::vector<Box> boxes;
};

#endif
