#ifndef CONFIG_SPACE_H
#define CONFIG_SPACE_H

#include "box.h"
#include <vector>

class ConfigSpace {
	public:
		void addBox(Box box);
	private:
		std::vector<Box> boxes;
};

#endif
