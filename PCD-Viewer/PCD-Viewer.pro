TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    pcd_helpers.cpp

INCLUDEPATH += /usr/include/pcl-1.8 \
    /usr/include/eigen3 \
    /usr/include/vtk/

LIBS +=  -lboost_system \
    -lboost_filesystem \
    -lvtkCommonDataModel \
    -lvtkCommonMath \
    -lvtkCommonCore \
    -lvtkRenderingCore \
    -lpcl_io \
    -lpcl_visualization \
    -lpcl_common

HEADERS += \
    pcd_helpers.h
