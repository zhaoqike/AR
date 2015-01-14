LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)


OPENCV_LIB_TYPE :=STATIC
include ../OpenCV-2.4.9-android-sdk/sdk/native/jni/OpenCV.mk

LOCAL_MODULE    := ar
LOCAL_SRC_FILES := ar.cpp \
ARDrawingContext.cpp \
ARPipeline.cpp \
Cameracalibration.cpp \
GeometryTypes.cpp \
Pattern.cpp \
PatternDetector.cpp \
Timer.cpp \
gl_code.cpp \
Utils.cpp \
Globals.cpp \
MutexImage.cpp \
Model/mesh.cpp \
Model/Model.cpp \
Model/pmesh.cpp \
Model/triangle.cpp \
Model/vec3.cpp \
Model/vertex.cpp \
cg/mypoint.cpp \
cg/myline.cpp \
cg/mypolygon.cpp



LOCAL_CPPFLAGS  += --std=c++11 -pthread -fexceptions -frtti -pthread \
-D__GXX_EXPERIMENTAL_CXX0X__ \
-D_GLIBCXX_HAS_GTHREADS \
-fpermissive

LOCAL_LDLIBS     += -llog -ldl -lGLESv1_CM  

include $(BUILD_SHARED_LIBRARY)
