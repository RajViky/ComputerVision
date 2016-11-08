TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp

INCLUDEPATH += /usr/include/pcl-1.8 \
    /usr/include/eigen3 \
    /usr/include/vtk/ \
    ../../rtab-map-fork/rtabmap/corelib/include/ \
    ../../rtab-map-fork/rtabmap/utilite/include/

LIBS +=  -lboost_system \
    -lboost_filesystem \
    -lvtkCommonDataModel \
    -lvtkCommonMath \
    -lvtkCommonCore \
    -lvtkRenderingCore \
    -lpcl_io \
    -lpcl_visualization \
    -lpcl_common \
    -lopencv_core \
    -L../../rtab-map-fork/rtabmap/bin/ \
    -lrtabmap_cored \
    -lrtabmap_guid \
    -lrtabmap_utilited
