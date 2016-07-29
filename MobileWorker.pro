TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
SOURCES += main.cpp

CONFIG += c++11
LIBS += -pthread

include(deployment.pri)
qtcAddDeployment()

HEADERS += \
    ekf_algorithm.h \
    encoder.h \
    global.h \
    myahrs_plus.hpp \
    sensor.h \
    serial.h \
    yr9010.h

