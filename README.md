KinectArm
=========

Using a Kinect sensor to monitor arm movements and update robotic arm accordingly


INSTALL
=======

1)	Install libfreenect, gfreenect, and Skeltrack as per these instructions:
http://tayyabnaseer.blogspot.com/2012/05/installing-skeltrack-on-ubuntu.html

	Note: I had trouble installing this on Ubuntu 13.10. If you do too, follow
	these instructions: http://openkinect.org/wiki/Getting_Started#Ubuntu_Manual_Install
	The important part for me was the bit after the "Quick copy-paste instructions to get
	up-and-running instantly:" code.

	If you're having trouble getting it to build, refer to the solution on this page:
	https://github.com/joaquimrocha/Skeltrack/issues/15

2)	Make sure your common.mk in /eecs467/src is unchanged from the eecs467 repository.
	To check this, run "git diff /eecs467/src/common.mk". If there's any output,
	run "git checkout /eecs467/src/common.mk"

3)	Run "make clean all"


NOTES
=====

Output binaries:

kinectarm_app: Accepts LCM messages on joint positions and actuates arm servos

kinectvision_app: 


Fakenect
========

To record a set of data simply run:

```
fakenect-record <directory-to-record>
```

After recording, you should be able to play it back like this using
glview:

```
LD_PRELOAD="/usr/local/lib64/fakenect/libfreenect.so" FAKENECT_PATH="<path-into-directory>" freenect-glview 
```

For some reason this doesn't work with the cpp wrapper, so we might
want to use the c version of the code.
