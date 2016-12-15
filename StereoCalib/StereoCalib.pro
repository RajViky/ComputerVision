TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp

LIBS += -lboost_system \
    -lboost_filesystem \
    -lboost_date_time \
    -lopencv_core \
    -lopencv_imgcodecs \
    -lopencv_imgproc \
    -lopencv_videoio \
    -lopencv_calib3d \
    -lopencv_highgui
