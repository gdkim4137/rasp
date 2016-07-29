TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

LIBS += -pthread

SOURCES += main.cpp


HEADERS += \
    myahrs_plus.hpp \
    sensor.h \
    serial.h \
    ekf_algorithm.h \
    yr9010.h \
    encoder.h \
    global.h
