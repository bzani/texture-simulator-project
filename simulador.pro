#-------------------------------------------------
#
# Project created by QtCreator 2016-03-24T19:03:49
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET    = simulador
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp \
    pso.cpp

INCLUDEPATH += /usr/local/include
LIBS += -L/usr/local/lib \
-lopencv_core -lopencv_imgproc -lopencv_highgui \
-lopencv_ml -lopencv_video -lopencv_features2d \
-lopencv_calib3d -lopencv_objdetect \
-lopencv_flann -lopencv_imgcodecs

HEADERS += \
    functions.h \
    psoconfig.h \
    pso.h \
    particula.h \
    enxame.h \
    libs.h \
    psoconfig.h \
    pso.h \
    particula.h \
    libs.h \
    functions.h \
    enxame.h
