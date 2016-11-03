TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp

LIBS += -lfreenect2 \
    -lglfw -lGL -lGLU -lusb \
    -lboost_system \
    -lboost_thread \
    -lboost_filesystem \
    -lboost_date_time \
    -L/opt/opencv3/lib \
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

QMAKE_RPATHDIR += ./
