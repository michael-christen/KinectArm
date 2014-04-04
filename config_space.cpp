#include "config_space.h"
#include "../common/matd.h"
#include "math.h"
#include <vector>
#include "bounding_box.h"

void ConfigSpace::addBoundingBox(BoundingBox box) {
	this->boxes.push_back(box);
}
