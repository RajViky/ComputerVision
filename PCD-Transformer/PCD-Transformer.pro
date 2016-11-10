TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    ../PCD-Viewer/pcd_helpers.cpp

INCLUDEPATH += /usr/include/pcl-1.8 \
    /usr/include/eigen3

LIBS +=  -lboost_system \
    -lboost_filesystem \
    -lpcl_io \
    -lpcl_common \
    -lpcl_filters

HEADERS += \
    ../PCD-Viewer/pcd_helpers.h
