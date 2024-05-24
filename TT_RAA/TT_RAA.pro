QT       += core gui opengl widgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    Dinamic.cpp \
    InverseKinematic.cpp \
    Kinematic.cpp \
    ax12a.cpp \
    body.cpp \
    controller.cpp \
    gl.cpp \
    joint.cpp \
    main.cpp \
    mainwindow.cpp \
    robot.cpp \
    trajectories.cpp \
    worldgl.cpp

HEADERS += \
    Dinamic.h \
    InverseKinematic.h \
    Kinematic.h \
    ax12a.h \
    body.h \
    controller.h \
    gl.h \
    joint.h \
    mainwindow.h \
    robot.h \
    trajectories.h \
    worldgl.h

FORMS += \
    mainwindow.ui

INCLUDEPATH += /home/shiza/DynamixelSDK/c++/include/dynamixel_sdk #Se incluye el directorio de la SDK de Dynamixel
LIBS  += -ldxl_x64_cpp
 -lrt
 -lQt5Widgets

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
