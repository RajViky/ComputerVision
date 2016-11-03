TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp

LIBS += -lfreenect2 \
    -lboost_system \
    -lboost_filesystem \
    -lboost_date_time \
    -lopencv_core \
    -lopencv_imgcodecs

QMAKE_RPATHDIR += ./
