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

#include "gl.h"
#include "robot.h"
#include "joint.h"
#include "body.h"
#include "worldgl.h"
#include "Kinematic.h"
#include "InverseKinematic.h"
#include "Dinamic.h"
#include "ax12a.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    float a,b,c,d;

    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void loadRobot();

public slots:

    void setRobotConfig();
    //void setSlidersLimits(vector<int> lMax, vector<int> lMin);


private slots:


    void on_eslabon1_sliderMoved(int position);

    void on_eslabon2_sliderMoved(int position);

    void on_eslabon3_sliderMoved(int position);

    void on_eslabon4_sliderMoved(int position);

    void on_xplus_clicked();

    void on_xminus_clicked();

    void on_yplus_clicked();

    void on_yminus_clicked();

    void on_zplus_clicked();

    void on_zminus_clicked();

    void on_home_clicked();

    void on_pestanas_tabBarClicked(int index);

private:
    Ui::MainWindow *ui;
    GL *gl;
    Robot *newRobot;
    Body *newBody;
    Joint *newJoint;
    WorldGL *objWorldGL;
    int adjust1, adjust2, adjust3, adjust4, adjust5;
};
#endif // MAINWINDOW_H
