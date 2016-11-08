TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp

LIBS += -lfreenect2 \
    -lglfw -lGL -lGLU -lusb
LIBS += -L/opt/opencv3/lib \
    -lopencv_videostab \
    -lopencv_superres \
    -lopencv_stitching \
    -lopencv_photo \
    -lopencv_objdetect \
    -lopencv_video \
    -lopencv_ml \
    -lopencv_calib3d \
    -lopencv_features2d \
    -lopencv_highgui \
    -lopencv_imgproc \
    -lopencv_flann \
    -lopencv_core \
    -lopencv_videoio \
    -lopencv_imgproc \
    -lopencv_imgcodecs

INCLUDEPATH += /usr/include/pcl-1.8 \
    /usr/include/eigen3 \
    /usr/include/vtk/

LIBS += -lboost_system \
    -lboost_thread \
    -L/home/beda/data/skola/_Oulu/PCL-libRealSense/pkg/pcl-librealsense-git/usr/lib \
    -lvtkCommonCore \
    -lvtkCommonExecutionModel \
    -lvtkCommonDataModel \
    -lvtkCommonMath \
    -lvtkRenderingLOD \
    -lvtkRenderingCore \
    -lvtkFiltersSources \
    -lpcl_common \
    -lpcl_filters \
    -lpcl_io \
    -lpcl_visualization \
    -lpcl_segmentation \
    -lpcl_search \
    -lpcl_features

