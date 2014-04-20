include ../common.mk

# Defined by Brian Daniels
# usr lib dirs
LDFLAGS_USRLIBDIRS := -L/usr/local/lib64/ -L/usr/local/lib/

# libfreenect
CFLAGS_LIBFREENECT := -I/usr/local/include/libfreenect
LDFLAGS_LIBFREENECT :=  -lfreenect

# gfreenect
CFLAGS_GFREENECT := -I/usr/local/include/gfreenect-0.1
LDFLAGS_GFREENECT :=  -lgfreenect-0.1

# skeltrack
CFLAGS_SKELTRACK := -I/usr/local/include/skeltrack-0.1
LDFLAGS_SKELTRACK :=  -lskeltrack-0.1

# clutter
CFLAGS_CLUTTER := `pkg-config clutter-1.0 --cflags`
LDFLAGS_CLUTTER := `pkg-config clutter-1.0 --libs`
# End defined by Brian Daniels

CFLAGS = $(CFLAGS_STD) $(CFLAGS_COMMON) $(CFLAGS_VX) $(CFLAGS_GTK) $(CFLAGS_LCMTYPES) $(CFLAGS_LCM) $(CFLAGS_VX) $(CFLAGS_LIBFREENECT) $(CFLAGS_GFREENECT) $(CFLAGS_SKELTRACK) $(CFLAGS_CLUTTER) -fPIC -O2 
LDFLAGS =  $(LDFLAGS_VX_GTK) $(LDFLAGS_VX) $(LDFLAGS_GTK) $(LDFLAGS_IMAGESOURCE) $(LDFLAGS_COMMON) $(LDFLAGS_LCMTYPES) $(LDFLAGS_LCM) $(LDFLAGS_STD) $(LDFLAGS_USRLIBDIRS) $(LDFLAGS_LIBFREENECT) $(LDFLAGS_GFREENECT) $(LDFLAGS_SKELTRACK) $(LDFLAGS_CLUTTER)
CXXFLAGS_STD := -g -D FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -D REENTRANT \
	-Wall -Wno-unused-parameter -pthread -Wno-write-strings -Wno-error=switch
CFLAGS_CXX = $(CXXFLAGS_STD) $(CFLAGS_COMMON) $(CFLAGS_VX) $(CFLAGS_GTK) $(CFLAGS_USB) $(CFLAGS_LIBFREENECT) -fPIC -O3 -std=c++0x -DNDEBUG

BINARIES = ../../bin/kinectarm_app ../../bin/kinectvision_app ../../bin/skeltrack_vision
LIB = ../../lib

all: $(BINARIES)

arm: ../../bin/kinectarm_app

vision: ../../bin/kinectvision_app

skeltrack: ../../bin/skeltrack_vision

../../bin/kinectarm_app: kinectarm_app.o arm_gui.o body_utility.o body.o pid_ctrl.o eecs467_util.o\
	skeleton_joint_t.o skeleton_joint_list_t.o config_space.o bounding_box.o rexarm.o\
	data_smoother.o state_machine.o gripper_lcm_t.o
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

../../bin/kinectvision_app: kinectvision_app.o vision_gui.o body.o disjoint.o gripper.o blob_detection.o\
	skeleton_joint_t.o skeleton_joint_list_t.o pixel.o eecs467_util.o kinect_handle.o image_helper.o filter.o \
	Gradient.o Line.o Image.o Graph.o joint.o gripper_lcm_t.o
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS) -O3

../../bin/skeltrack_vision: skeltrack_vision.o skeleton_joint_t.o skeleton_joint_list_t.o gripperc.o gripper_lcm_t.o
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(BINARIES)
