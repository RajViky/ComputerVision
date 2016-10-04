TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp

INCLUDEPATH += /home/beda/data/skola/_Oulu/PCL-libRealSense/pkg/pcl-librealsense-git/usr/include/pcl-1.8 \
    /usr/include/eigen3/ \
    /usr/include/vtk/

QMAKE_FLAGS += "-rpath=/home/beda/data/skola/_Oulu/PCL-libRealSense/pkg/pcl-librealsense-git/usr/lib/"

LIBS += -lboost_system \
    -lboost_thread \
    /home/beda/data/skola/_Oulu/PCL-libRealSense/pkg/pcl-librealsense-git/usr/lib/libpcl_io.so.1.8.0 \
    /home/beda/data/skola/_Oulu/PCL-libRealSense/pkg/pcl-librealsense-git/usr/lib/libpcl_visualization.so.1.8.0
