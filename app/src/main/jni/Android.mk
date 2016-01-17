LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

include $(call all-subdir-makefiles)

#opencv
OPENCVROOT:= /home/aravind/Downloads/OpenCV-android-sdk
OPENCV_CAMERA_MODULES:=on
OPENCV_INSTALL_MODULES:=on
OPENCV_LIB_TYPE:=SHARED

TESSERACT_PATH := $(LOCAL_PATH)/com_googlecode_tesseract_android
LEPTONICA_PATH := $(LOCAL_PATH)/com_googlecode_leptonica_android
LIBPNG_PATH := $(LOCAL_PATH)/libpng

# include ${LIBPNG_PATH}/Android.mk
# include ${LEPTONICA_PATH}/Android.mk
# include ${TESSERACT_PATH}/Android.mk

include ${OPENCVROOT}/sdk/native/jni/OpenCV.mk

LOCAL_SRC_FILES := main.cpp \
    precomp.cpp \
    erfilter.cpp \

LOCAL_LDLIBS += -llog
LOCAL_MODULE := text

include $(BUILD_SHARED_LIBRARY)