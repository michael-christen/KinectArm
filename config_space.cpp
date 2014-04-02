#include "config_space.h"
#include "../common/matd.h"
#include "math.h"
#include <vector>
#include "box.h"

void ConfigSpace::addBox(Box box) {
	this->boxes.push_back(box);
}
