TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp

INCLUDEPATH += /usr/include/pcl-1.8 \
    /usr/include/eigen3 \
    /usr/include/vtk/

LIBS += -lboost_system \
    -lboost_thread \
    -L/home/beda/data/skola/_Oulu/PCL-libRealSense/pkg/pcl-librealsense-git/usr/lib \
    -lpcl_io \
    -lpcl_visualization


#QMAKE_RPATHDIR += /home/beda/data/skola/_Oulu/PCL-libRealSense/pkg/pcl-librealsense-git/usr/lib
