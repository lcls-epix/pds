libnames := camera

libsrcs_camera := DmaSplice.cc \
		  FrameHandle.cc \
		  Frame.cc \
		  LvCamera.cc \
		  PicPortCL.cc \
		  TwoDMoments.cc \
		  TwoDGaussian.cc \
	          FexFrameServer.cc \
		  Opal1kCamera.cc \
	          TM6740Camera.cc \
		  CameraManager.cc \
		  Opal1kManager.cc \
	          TM6740Manager.cc
#	          FrameServer.cc \

libsinc_camera := /usr/include/lvsds
libincs_camera := leutron/include

#tgtnames := camreceiver
#tgtnames += camsend
tgtnames :=

ifneq ($(findstring -opt,$(tgt_arch)),)
tgtnames := camsend camreceiver serialcmd
endif
tgtnames := camsend camreceiver serialcmd

#ifeq ($(shell uname -m | egrep -c '(x86_|amd)64$$'),1)
#ARCHCODE=64
#else
ARCHCODE=32
#endif

leutron_libs := leutron/lvsds.34.${ARCHCODE}
leutron_libs += leutron/LvCamDat.34.${ARCHCODE}
leutron_libs += leutron/LvSerialCommunication.34.${ARCHCODE}

tgtsrcs_camsend := camsend.cc
tgtlibs_camsend := pds/service pds/collection pds/utility pds/config pds/camera pds/client pds/xtc
tgtlibs_camsend += pds/vmon pds/mon
tgtlibs_camsend += pdsdata/xtcdata pdsdata/camdata pdsdata/opal1kdata pdsdata/pulnixdata
tgtincs_camsend := pds/zerocopy/kmemory pds/camera
tgtlibs_camsend += $(leutron_libs)
tgtincs_camsend += leutron/include

tgtsrcs_camreceiver := camreceiver.c display.cc
tgtincs_camreceiver := qt/include

tgtlibs_camreceiver := qt/QtGui qt/QtCore
#tgtslib_camreceiver := /usr/lib/qt4/lib/QtGui /usr/lib/qt4/lib/QtCore
#tgtsinc_camreceiver := /usr/lib/qt4/include

tgtsrcs_serialcmd := serialcmd.cc
tgtlibs_serialcmd := $(leutron_libs)
tgtincs_serialcmd := leutron/include

