#QT -= gui

QT       += core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11 console
CONFIG -= app_bundle



# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    GUI/component.cpp \
    GUI/main_widget.cpp \
    GUI/start_widget.cpp \
    detector.cpp \
    main.cpp \
    capture_video.cpp \
    armor_detect.cpp \
    solve_angle.cpp \
    serial.cpp \
    utils.cpp \
    kalman.cpp \
    buff_detect.cpp \
    thread_task.cpp \
    settings.cpp \
    ui_tool.cpp \
    top_detect.cpp

HEADERS += \
    GUI/component.h \
    GUI/main_widget.h \
    GUI/start_widget.h \
    capture_video.h \
    armor_detect.h \
    solve_angle.h \
    settings.h \
    serial.h \
    kalman.hpp \
    thread_task.h \
    utils.h \
    galaxy/DxImageProc.h \
    galaxy/GxIAPI.h \
    buff_detect.h \
    detector.h \
    stdafx.h \
    ui_tool.h \
    top_detect.h

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

DISTFILES += \
    debug_record.txt

#LIBS += -L /usr/lib -lcurses

CONFIG += precompile_header
PRECOMPILED_HEADER = stdafx.h
# 0 for Linux, 1 for Windows
DEFINES += OS=0 #\
#           GUI

#---------------OPENCV相关配置---------------

INCLUDEPATH += /usr/local/include \
#              /usr/local/include/opencv \
               /usr/local/include/opencv4/opencv2 \

LIBS += /usr/local/lib/libopencv_highgui.so \
        /usr/local/lib/libopencv_core.so    \
        /usr/local/lib/libopencv_imgproc.so \
        /usr/local/lib/libopencv_imgcodecs.so \
        #/usr/local/lib/..so \
        -pthread
      # -ldl
LIBS += -L/usr/local/lib \
#        -lopencv_shape \
        -lopencv_stitching \
#        -lopencv_superres \
#        -lopencv_videostab \
        -lopencv_calib3d \
        -lopencv_features2d \
        -lopencv_objdetect \
        -lopencv_highgui \
        -lopencv_videoio \
        -lopencv_photo \
        -lopencv_imgcodecs \
        -lopencv_video \
        -lopencv_ml \
        -lopencv_imgproc \
        -lopencv_flann \
        -lopencv_core \
        -lopencv_dnn

#---------------工业驱动相关配置---------------

LIBS += /usr/lib/libgxiapi.so \
        /usr/lib/libdximageproc.so

#INCLUDEPATH += /home/nvidia/Galaxy_V0.0.21_Linux_armv8_CN/inc
#INCLUDEPATH += /home/jia/SoftWare/dhcam_install_20181107/dh_camera/daheng-sdk-x64/sdk/genicam/library/CPP/include
#INCLUDEPATH += /home/jia/SoftWare/dhcam_install_20181107/dh_camera/daheng-sdk-x64/sdk/include
INCLUDEPATH += /home/coscj/dhcam_install_20181107/dh_camera/daheng-sdk-x64/sdk/genicam/library/CPP/include \
                /home/coscj/dhcam_install_20181107/dh_camera/daheng-sdk-x64/sdk/include

#SUBDIRS += \
#    GUI/ACEVisionUI.pro
