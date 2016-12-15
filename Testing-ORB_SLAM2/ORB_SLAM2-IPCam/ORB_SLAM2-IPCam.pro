TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp

DEFINES += COMPILEDWITHC11

INCLUDEPATH += /usr/include/opencv2 \
    /home/beda/data/skola/_Oulu/ORB_SLAM2/ \
    /home/beda/data/skola/_Oulu/ORB_SLAM2/include/ \
    /usr/include/eigen3 \
    /opt/pangolin/usr/local/include/

LIBS += -L/home/beda/data/skola/_Oulu/ORB_SLAM2/lib/ \
    -lORB_SLAM2 \
    -lopencv_videostab \
    -lopencv_superres \
    -lopencv_stitching \
    -lopencv_photo \
    -lopencv_objdetect \
    -lopencv_video \
    -lopencv_videoio \
    -lopencv_ml \
    -lopencv_calib3d \
    -lopencv_features2d \
    -lopencv_highgui \
    -lopencv_imgproc \
    -lopencv_flann \
    -lopencv_core \
    -ldl -lm -ltbb \
    -L/opt/pangolin/usr/local/lib \
    -lpangolin \
    -lpthread -lrt -lGLU -lGL -lGLEW -lSM -lICE -lX11 -lXext -lpython3.5m -ldc1394 -lavcodec -lavformat -lavutil -lswscale -lpng -lz -ljpeg -ltiff -lIlmImf \
    -L/home/beda/data/skola/_Oulu/ORB_SLAM2/Thirdparty/DBoW2/lib \
    -lDBoW2 \
    -L//home/beda/data/skola/_Oulu/ORB_SLAM2/Thirdparty/g2o/lib \
    -lg2o
QMAKE_RPATHDIR += /home/beda/data/skola/_Oulu/ORB_SLAM2/lib \
    /opt/pangolin/usr/local/lib \
    /home/beda/data/skola/_Oulu/ORB_SLAM2/Thirdparty/DBoW2/lib \
    /home/beda/data/skola/_Oulu/ORB_SLAM2/Thirdparty/g2o/lib
