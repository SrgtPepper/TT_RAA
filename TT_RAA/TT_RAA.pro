QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    Dinamic.cpp \
    body.cpp \
    joint.cpp \
    main.cpp \
    mainwindow.cpp \
    opt1.cpp \
    opt2.cpp \
    opt3.cpp

HEADERS += \
    Dinamic.h \
    body.h \
    joint.h \
    mainwindow.h \
    opt1.h \
    opt2.h \
    opt3.h

FORMS += \
    mainwindow.ui \
    opt1.ui \
    opt2.ui \
    opt3.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
