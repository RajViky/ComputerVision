TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp

INCLUDEPATH += /opt/opencv3/include

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

QMAKE_RPATHDIR += /opt/opencv3/lib


    #-lopencv_ts \
    #-lopencv_contrib \
    #-lopencv_nonfree \
    #-lopencv_ocl \
    #-lopencv_gpu \
    #-lopencv_legacy \
