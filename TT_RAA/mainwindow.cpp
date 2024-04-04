#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>
#include <qthread.h>
#include <math.h>
#include "Kinematic.h"
#include "InverseKinematic.h"
#include "Dinamic.h"
#include "ax12a.h"
#define PI 3.141594

QCoreApplication *app = QCoreApplication::instance();

Kinematic k1;
InverseKinematic Invk1;

int contadorx = 1;
int contadory = 1;
int contadorz = 1;

ax12a m(0);
ax12a m1(1);
ax12a m2(2);
ax12a m3(3);

int home[4] = {512, 204, 512, 512};
int speed;

double rad = (PI/180);
double grad = 0.29297;
double q1, q2, q3, q4;
int px, py, pz, q1_inv, q2_inv, q3_inv, q1motor, q2motor, q3motor;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //app->processEvents();


    //Iniciando puerto
    m.initport();
    m1.initport();
    m2.initport();
    m3.initport();

    //Se encienden los pares
    m.enabletorque();
    m1.enabletorque();
    m2.enabletorque();
    m3.enabletorque();

    //Se restringe la velocidad de cada motor
    m.setspeed(speed = 165);
    m1.setspeed(speed = 165);
    m2.setspeed(speed = 165);
    m3.setspeed(speed = 165);

    //Se mueven los eslabones a la posicion inicial
    m.setposition(home[0]);
    m1.setposition(home[1]);
    m2.setposition(home[2]);
    m3.setposition(home[3]);

    //Inicia los sliders en la posición inicial
    ui->eslabon1->setValue(home[0]);
    ui->eslabon2->setValue(home[1]);
    ui->eslabon3->setValue(home[2]);
    ui->eslabon4->setValue(home[3]);

    //Obtiene los valores de posicion y los muestra en los displays



}

MainWindow::~MainWindow()
{
    delete ui;

    m.turnoff();
    m1.turnoff();
    m2.turnoff();
    m3.turnoff();
}

//Programación de los sliders del modo de operación por eslabón

void MainWindow::on_eslabon1_sliderMoved(int position)
{
    //Se manda la posicion al motor para que se mueva
    m.setposition(position);

    //Cada que se mueva el slider se leen las posiciones de los motores
    m.getposition();
    m1.getposition();
    m2.getposition();
    m3.getposition();

    //Se asignan las posiciones a variables para la cinematica directa
    q1 = (m.dxl_present_position*grad-150)*rad;
    q2 = (m1.dxl_present_position*grad-150+90)*rad;
    q3 = (m2.dxl_present_position*grad-150)*rad;
    q4 = (m3.dxl_present_position*grad-150)*rad;

    //Se introducen estos valores al metodo de la cinematica directa
    k1.DirecKinect(q1,q2,q3,k1.px,k1.py,k1.pz);

    Invk1.InverseKinect(k1.px, k1.py, k1.pz, Invk1.q1, Invk1.q2, Invk1.q3);



    //Se muestran los valores de posicion recibidos de la cinematica
    ui->pos_x_eslabon->display(k1.px);
    ui->pos_y_eslabon->display(k1.py);
    ui->pos_z_eslabon->display(k1.pz);

    //Se muestra el valor del ángulo
    ui->q1_eslabon->display((position*grad)-(home[0]*grad));


    ui->test1->display(Invk1.q1/rad);
    ui->test2->display(q2_inv);
    ui->test3->display(Invk1.q3/rad);

}


void MainWindow::on_eslabon2_sliderMoved(int position)
{
    //Se manda la posicion al motor para que se mueva
    m1.setposition(position);

    //Cada que se mueva el slider se leen las posiciones de los motores
    m.getposition();
    m1.getposition();
    m2.getposition();
    m3.getposition();

    //Se asignan las posiciones a variables para la cinematica directa
    q1 = (m.dxl_present_position*grad-150)*rad;
    q2 = (m1.dxl_present_position*grad-150+90)*rad;
    q3 = (m2.dxl_present_position*grad-150)*rad;
    q4 = (m3.dxl_present_position*grad-150)*rad;

    //Se introducen estos valores al metodo de la cinematica directa
    k1.DirecKinect(q1,q2,q3,k1.px,k1.py,k1.pz);

    Invk1.InverseKinect(k1.px, k1.py, k1.pz, Invk1.q1, Invk1.q2, Invk1.q3);

    //Se muestran los valores de posicion recibidos de la cinematica
    ui->pos_x_eslabon->display(k1.px);
    ui->pos_y_eslabon->display(k1.py);
    ui->pos_z_eslabon->display(k1.pz);

    //Se muestra el valor del ángulo
    ui->q2_eslabon->display((position*grad)-((home[1]+308)*grad));

    ui->test1->display(Invk1.q1/rad);
    ui->test2->display(Invk1.q2/rad);
    ui->test3->display(Invk1.q3/rad);
}


void MainWindow::on_eslabon3_sliderMoved(int position)
{
    //Se manda la posicion al motor para que se mueva
    m2.setposition(position);

    //Cada que se mueva el slider se leen las posiciones de los motores
    m.getposition();
    m1.getposition();
    m2.getposition();
    m3.getposition();

    //Se asignan las posiciones a variables para la cinematica directa
    q1 = (m.dxl_present_position*grad-150)*rad;
    q2 = (m1.dxl_present_position*grad-150+90)*rad;
    q3 = (m2.dxl_present_position*grad-150)*rad;
    q4 = (m3.dxl_present_position*grad-150)*rad;

    //Se introducen estos valores al metodo de la cinematica directa
    k1.DirecKinect(q1,q2,q3,k1.px,k1.py,k1.pz);

    Invk1.InverseKinect(k1.px, k1.py, k1.pz, Invk1.q1, Invk1.q2, Invk1.q3);

    //Se muestran los valores de posicion recibidos de la cinematica
    ui->pos_x_eslabon->display(k1.px);
    ui->pos_y_eslabon->display(k1.py);
    ui->pos_z_eslabon->display(k1.pz);

    //Se muestra el valor del ángulo
    ui->q3_eslabon->display((position*grad)-(home[2]*grad));

    ui->test1->display(Invk1.q1/rad);
    ui->test2->display(Invk1.q2/rad);
    ui->test3->display(Invk1.q3/rad);
}


void MainWindow::on_eslabon4_sliderMoved(int position)
{
    //Se manda la posicion al motor para que se mueva
    m3.setposition(position);

    //Cada que se mueva el slider se leen las posiciones de los motores
    m.getposition();
    m1.getposition();
    m2.getposition();
    m3.getposition();

    //Se asignan las posiciones a variables para la cinematica directa
    q1 = (m.dxl_present_position*grad-150)*rad;
    q2 = (m1.dxl_present_position*grad-150+90)*rad;
    q3 = (m2.dxl_present_position*grad-150)*rad;
    q4 = (m3.dxl_present_position*grad-150)*rad;

    //Se introducen estos valores al metodo de la cinematica directa
    k1.DirecKinect(q1,q2,q3,k1.px,k1.py,k1.pz);

    Invk1.InverseKinect(k1.px, k1.py, k1.pz, Invk1.q1, Invk1.q2, Invk1.q3);

    //Se muestran los valores de posicion recibidos de la cinematica
    ui->pos_x_eslabon->display(k1.px);
    ui->pos_y_eslabon->display(k1.py);
    ui->pos_z_eslabon->display(k1.pz);

    //Se muestra el valor del ángulo
    ui->q4_eslabon->display((position*grad)-(home[3]*grad));

    ui->test1->display(Invk1.q1/rad);
    ui->test2->display(Invk1.q2/rad);
    ui->test3->display(Invk1.q3/rad);
}



//Programación de los botones para el modo de operación por espacio de la herramienta

void MainWindow::on_xplus_clicked()
{
    //Cada que se mueva el slider se leen las posiciones de los motores
    m.getposition();
    m1.getposition();
    m2.getposition();
    m3.getposition();

    //Se asignan las posiciones a variables para la cinematica directa
    q1 = (m.dxl_present_position*grad-150)*rad;
    q2 = (m1.dxl_present_position*grad-150+90)*rad;
    q3 = (m2.dxl_present_position*grad-150)*rad;
    q4 = (m3.dxl_present_position*grad-150)*rad;

    //Se introducen estos valores al metodo de la cinematica directa
    k1.DirecKinect(q1,q2,q3,k1.px,k1.py,k1.pz);

    //Aumenta el valor de contador y se suma al valor de salida de la cinemática del eje correspondiente
    contadorx = contadorx + 1;
    px = k1.px + contadorx;

    //Se introducen los valores de px y las salidas de Y y Z de la cinematica directa en la cinematica inversa
    Invk1.InverseKinect(k1.px, k1.py, k1.pz, Invk1.q1, Invk1.q2, Invk1.q3);

    //Se convierten los angulos de radianes a las unidades del motor
    q1_inv = abs((Invk1.q1/rad)/grad)+512;
    q2_inv = abs((Invk1.q2/rad)/grad)+512-204;
    q3_inv = abs((Invk1.q3/rad)/grad)+512;

    m.setposition(q1_inv);
    m1.setposition(q2_inv);
    m2.setposition(q3_inv);

    ui->pos_x_herramienta->display(k1.px);
    ui->pos_y_herramienta->display(k1.py);
    ui->pos_z_herramienta->display(k1.pz);

    ui->q1_herramienta->display(q1_inv);
    ui->q2_herramienta->display(q2_inv);
    ui->q3_herramienta->display(q3_inv);

    ui->test1->display(Invk1.q1);
    ui->test2->display(Invk1.q2);
    ui->test3->display(Invk1.q3);
}


void MainWindow::on_xminus_clicked()
{

}


void MainWindow::on_yplus_clicked()
{

}


void MainWindow::on_yminus_clicked()
{

}


void MainWindow::on_zplus_clicked()
{

}


void MainWindow::on_zminus_clicked()
{

}


void MainWindow::on_home_clicked()
{
    //Se regresa a todos los motores a posición de home
    m.setposition(home[0]);
    m1.setposition(home[1]);
    m2.setposition(home[2]);
    m3.setposition(home[3]);

    //Se reinician los valores de los sliders
    ui->eslabon1->setValue(home[0]);
    ui->eslabon2->setValue(home[1]);
    ui->eslabon3->setValue(home[2]);
    ui->eslabon4->setValue(home[3]);
}

