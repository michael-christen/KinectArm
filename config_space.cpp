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

void ConfigSpace::addBoundingBox(BoundingBox &box) {
	this->boxes.push_back(box);
}

void ConfigSpace::draw(vx_buffer *buf, const float color[]) {
    std::vector<BoundingBox>::iterator it = this->boxes.begin();
    vx_object_t *box;
    for (; it != this->boxes.end(); it++) {
        box = vxo_chain(
		    // Base
		    vxo_mat_scale3(CM_TO_VX, CM_TO_VX, CM_TO_VX),
		    vxo_mat_translate3(it->getX(), it->getY(), it->getZ()),
		    vxo_mat_scale3(it->getW(), it->getH(), it->getD()),
		    vxo_box(vxo_mesh_style(color), vxo_lines_style(vx_yellow, 2.0f))
		);
		
		vx_buffer_add_back(buf, box);
    }
    
}
