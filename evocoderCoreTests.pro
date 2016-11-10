#-------------------------------------------------
#
# Project created by QtCreator 2016-11-02T13:12:22
#
#-------------------------------------------------

QT       += testlib

QT       -= gui

TARGET = tst_graph
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

QMAKE_CXXFLAGS += -bigobj

SOURCES += tst_graph.cpp
DEFINES += SRCDIR=\\\"$$PWD/\\\"

INCLUDEPATH += X:/boost_1_61_0
INCLUDEPATH += X:/EvoCoreLibrary/include
INCLUDEPATH += X:/EvoCoreLibrary/lib
INCLUDEPATH += C:/Python27/include

LIBS += -L$$quote(X:/boost_1_61_0/stage/lib) -lboost_python-vc140-mt-1_61
LIBS += -L$$quote(C:/Python27/libs)
LIBS += -L$$quote(X:/EvoCoreLibrary/lib) -lEvoCoderCore

RESOURCES += \
    extra_resources.qrc

