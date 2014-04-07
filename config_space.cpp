// General
#include <vector>

// VX
#include "vx/vx.h"
#include "vx/vxo_drawables.h"

// Local
#include "config_space.h"
#include "../common/matd.h"
#include "bounding_box.h"
#include "arm_gui.h"

bool ConfigSpace::testCollisions(BoundingBox box[], int num) {
    std::vector<BoundingBox*>::iterator it = this->boxes.begin();
    for (; it != this->boxes.end(); it++) {
    	for (int i = 0; i < num; i++) {
    		if ((*it)->intersect(&box[i])) {
    			return true;
    		}
    	}
    }

    return false;
}

void ConfigSpace::addBoundingBox(BoundingBox *box) {
	this->boxes.push_back(box);
}

void ConfigSpace::draw(vx_buffer *buf, const float color[]) {
    std::vector<BoundingBox*>::iterator it = this->boxes.begin();
    for (; it != this->boxes.end(); it++) {
        (*it)->draw(buf, color);
    }
    
}
