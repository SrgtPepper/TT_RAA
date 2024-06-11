#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QString>
#include <QTextStream>
#include <QIODevice>
#include <QtWidgets>
#include <QMainWindow>
#include <stdlib.h>
#include <vector>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <qthread.h>
#define _USE_MATH_DEFINES
#include <cmath>
#include <QPixmap>
#include <QTimer>

#include "gl.h"
#include "robot.h"
#include "joint.h"
#include "body.h"
#include "worldgl.h"
#include "myworker.h"
#include "ax12a.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class Body;
class Joint;
class Robot;
class WorldGL;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void loadRobot();

public slots:

    void setRobotConfig(double q1, double q2, double q3, double q4);

    void handleResults(double q1, double q2, double q3, double q4);

    void handleSlidersValues(int s1, int s2, int s3, int s4);

    void toObtainGraphs();

    void turnOffAllMotors();

private slots:

    void on_xplus_clicked();

    void on_xminus_clicked();

    void on_yplus_clicked();

    void on_yminus_clicked();

    void on_zplus_clicked();

    void on_zminus_clicked();

    void on_home_clicked();

    void on_eslabon1_valueChanged(int value);

    void on_eslabon2_valueChanged(int value);

    void on_eslabon3_valueChanged(int value);

    void on_eslabon4_valueChanged(int value);

    void on_checkBox_stateChanged(int arg1);

    void on_pestanas_tabBarClicked(int index);

    void on_checkBox_2_stateChanged(int arg1);

    void on_testModeButton_clicked();

signals:
    void pestanaClicked(int index);
    void goHome();
    void checkboxControlStateChanged(int state);
    void checkboxFileStateChanged(int state);

private:
    Ui::MainWindow *ui;
    QThread *workerThread;
    MyWorker *worker;
    GL *gl;
    Robot *newRobot;
    Body *newBody;
    Joint *newJoint;
    WorldGL *objWorldGL;
};
#endif // MAINWINDOW_H
