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
    -lvtkCommonCore \
    -lvtkCommonDataModel \
    -lvtkCommonMath \
    -lvtkRenderingCore \
    -lpcl_common \
    -lpcl_filters \
    -lpcl_io \
    -lpcl_visualization \
    -lpcl_segmentation \
    -lpcl_search \
    -lpcl_features
