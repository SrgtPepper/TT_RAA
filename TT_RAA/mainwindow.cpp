#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>
#include <qthread.h>
#define _USE_MATH_DEFINES
#include <cmath>
#include "Kinematic.h"
#include "InverseKinematic.h"
#include "Dinamic.h"
#include "ax12a.h"

QCoreApplication *app = QCoreApplication::instance();

Kinematic k1,k2;
InverseKinematic Ik1, Ik2;

int pxcont = 0;
int pycont = 0;
int pzcont = 0;

ax12a m(0);
ax12a m1(1);
ax12a m2(2);
ax12a m3(3);

int home[4] = {512, 204, 512, 512};
int speed;

double rad = (M_PI/180);
double grad = 0.29297;
double q1, q2, q3, q4;
double q_sliders[4] = {512, 204, 512, 512};
int px, py, pz, q1inv, q2inv, q3inv, q1motor, q2motor, q3motor;

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
    q_sliders[0] = (position*grad-150)*rad;
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

    //Se muestran los valores de posicion recibidos de la cinematica
    ui->pos_x_eslabon->display(k1.px);
    ui->pos_y_eslabon->display(k1.py);
    ui->pos_z_eslabon->display(k1.pz);

    //Se muestra el valor del ángulo
    ui->q1_eslabon->display((position*grad)-(home[0]*grad));
}


void MainWindow::on_eslabon2_sliderMoved(int position)
{
    q_sliders[1] = (position*grad-150+90)*rad;

    //Se revisa que el robot no vaya a entrar en singularidad
    k1.hitbox(k1.px,k1.py,k1.pz,k1.hit);
    k2.DirecKinect(q_sliders[0],q_sliders[1],q_sliders[2],k2.px,k2.py,k2.pz);
    k2.hitbox(k2.px,k2.py,k2.pz,k2.hit);

    //Se manda la posicion al motor para que se mueva
    if(!(k1.hit && k2.hit)){
        m1.setposition(position);
    }

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

    //Se muestran los valores de posicion recibidos de la cinematica
    ui->pos_x_eslabon->display(k1.px);
    ui->pos_y_eslabon->display(k1.py);
    ui->pos_z_eslabon->display(k1.pz);

    //Se muestra el valor del ángulo
    ui->q2_eslabon->display((position*grad)-((home[1]+308)*grad));
}


void MainWindow::on_eslabon3_sliderMoved(int position)
{

    q_sliders[2] = (position*grad-150)*rad;

    //Se revisa que el robot no vaya a entrar en singularidad
    k1.hitbox(k1.px,k1.py,k1.pz,k1.hit);
    k2.DirecKinect(q_sliders[0],q_sliders[1],q_sliders[2],k2.px,k2.py,k2.pz);
    k2.hitbox(k2.px,k2.py,k2.pz,k2.hit);

    //Se manda la posicion al motor para que se mueva
    if(!(k1.hit && k2.hit)){
        m2.setposition(position);
    }

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
    k1.hitbox(k1.px,k1.py,k1.pz,k1.hit);
    ui->test1->display(k1.hit);

    //Se muestran los valores de posicion recibidos de la cinematica
    ui->pos_x_eslabon->display(k1.px);
    ui->pos_y_eslabon->display(k1.py);
    ui->pos_z_eslabon->display(k1.pz);

    //Se muestra el valor del ángulo
    ui->q3_eslabon->display((position*grad)-(home[2]*grad));
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
    k1.hitbox(k1.px,k1.py,k1.pz,k1.hit);
    ui->test1->display(k1.hit);

    //Se muestran los valores de posicion recibidos de la cinematica
    ui->pos_x_eslabon->display(k1.px);
    ui->pos_y_eslabon->display(k1.py);
    ui->pos_z_eslabon->display(k1.pz);

    //Se muestra el valor del ángulo
    ui->q4_eslabon->display((position*grad)-(home[3]*grad));
}



//Programación de los botones para el modo de operación por espacio de la herramienta

void MainWindow::on_xplus_clicked()
{
    //Se revisa que el robot no vaya a entrar en singularidad
    Ik1.hitbox(pxcont, pycont, pzcont, Ik1.hit);

    //Incrementa en uno el valor de contador
    pxcont = pxcont + 1;

    //Entran los valores de los contadores al metodo y salen los angulos
    Ik1.InverseKinect(pxcont, pycont, pzcont, Ik1.q1, Ik1.q2, Ik1.q3);

    //Se convierten los valores para entrar a los motores
    q1inv = (Ik1.q1/rad);
    q2inv = (Ik1.q2/rad);
    q3inv = (Ik1.q3/rad);

    q1motor = (q1inv/grad);
    q2motor = (q2inv/grad);
    q3motor = (q3inv/grad);

    //Se mandan los valores a los motores
    m.setposition(q1motor);
    m1.setposition(q2motor);
    m2.setposition(q3motor);

    //Por el momento se muestran en los displays extra los valores de q
    ui->test1->display(q1inv);
    ui->test2->display(q2inv);
    ui->test3->display(q3inv);

    ui->pos_x_herramienta->display(pxcont);
}


void MainWindow::on_xminus_clicked()
{
    //Incrementa en uno el valor de contador
    pxcont--;

    //Entran los valores de los contadores al metodo y salen los angulos
    Ik1.InverseKinect(pxcont, pycont, pzcont, Ik1.q1, Ik1.q2, Ik1.q3);

    //Se convierten los valores para entrar a los motores
    q1inv = (Ik1.q1/rad);
    q2inv = (Ik1.q2/rad);
    q3inv = (Ik1.q3/rad);

    q1motor = (q1inv/grad);
    q2motor = (q2inv/grad);
    q3motor = (q3inv/grad);

    //Se mandan los valores a los motores
    m.setposition(q1motor);
    m1.setposition(q2motor);
    m2.setposition(q3motor);

    //Por el momento se muestran en los displays extra los valores de q
    ui->test1->display(q1inv);
    ui->test2->display(q2inv);
    ui->test3->display(q3inv);

    ui->pos_x_herramienta->display(pxcont);
}


void MainWindow::on_yplus_clicked()
{
    //Incrementa en uno el valor de contador
    pycont++;

    //Entran los valores de los contadores al metodo y salen los angulos
    Ik1.InverseKinect(pxcont, pycont, pzcont, Ik1.q1, Ik1.q2, Ik1.q3);

    //Se convierten los valores para entrar a los motores
    q1inv = (Ik1.q1/rad);
    q2inv = (Ik1.q2/rad);
    q3inv = (Ik1.q3/rad);

    q1motor = (q1inv/grad);
    q2motor = (q2inv/grad);
    q3motor = (q3inv/grad);

    //Se mandan los valores a los motores
    m.setposition(q1motor);
    m1.setposition(q2motor);
    m2.setposition(q3motor);

    //Por el momento se muestran en los displays extra los valores de q
    ui->test1->display(q1inv);
    ui->test2->display(q2inv);
    ui->test3->display(q3inv);

    ui->pos_y_herramienta->display(pycont);
}


void MainWindow::on_yminus_clicked()
{
    //Incrementa en uno el valor de contador
    pycont--;

    //Entran los valores de los contadores al metodo y salen los angulos
    Ik1.InverseKinect(pxcont, pycont, pzcont, Ik1.q1, Ik1.q2, Ik1.q3);

    //Se convierten los valores para entrar a los motores
    q1inv = (Ik1.q1/rad);
    q2inv = (Ik1.q2/rad);
    q3inv = (Ik1.q3/rad);

    q1motor = (q1inv/grad);
    q2motor = (q2inv/grad);
    q3motor = (q3inv/grad);

    //Se mandan los valores a los motores
    m.setposition(q1motor);
    m1.setposition(q2motor);
    m2.setposition(q3motor);

    //Por el momento se muestran en los displays extra los valores de q
    ui->test1->display(q1inv);
    ui->test2->display(q2inv);
    ui->test3->display(q3inv);

    ui->pos_y_herramienta->display(pycont);
}


void MainWindow::on_zplus_clicked()
{
    //Incrementa en uno el valor de contador
    pzcont++;

    //Entran los valores de los contadores al metodo y salen los angulos
    Ik1.InverseKinect(pxcont, pycont, pzcont, Ik1.q1, Ik1.q2, Ik1.q3);

    //Se convierten los valores para entrar a los motores
    q1inv = (Ik1.q1/rad);
    q2inv = (Ik1.q2/rad);
    q3inv = (Ik1.q3/rad);

    q1motor = abs(q1inv/grad);
    q2motor = abs(q2inv/grad);
    q3motor = abs(q3inv/grad);

    //Se mandan los valores a los motores
    m.setposition(q1motor);
    m1.setposition(q2motor);
    m2.setposition(q3motor);

    //Por el momento se muestran en los displays extra los valores de q
    ui->test1->display(q1inv);
    ui->test2->display(q2inv);
    ui->test3->display(q3inv);

    ui->pos_z_herramienta->display(pzcont);
}


void MainWindow::on_zminus_clicked()
{
    //Incrementa en uno el valor de contador
    pzcont--;

    //Entran los valores de los contadores al metodo y salen los angulos
    Ik1.InverseKinect(pxcont, pycont, pzcont, Ik1.q1, Ik1.q2, Ik1.q3);

    //Se convierten los valores para entrar a los motores
    q1inv = (Ik1.q1/rad);
    q2inv = (Ik1.q2/rad) + 90;
    q3inv = (Ik1.q3/rad);

    q1motor = (q1inv/grad);
    q2motor = (q2inv/grad);
    q3motor = (q3inv/grad);

    //Se mandan los valores a los motores
    m.setposition(q1motor);
    m1.setposition(q2motor);
    m2.setposition(q3motor);

    //Por el momento se muestran en los displays extra los valores de q
    ui->test1->display(q1inv);
    ui->test2->display(q2inv);
    ui->test3->display(q3inv);

    ui->pos_z_herramienta->display(pzcont);
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


void MainWindow::on_pestanas_tabBarClicked(int index)
{
    int maux, m1aux, m2aux;
    double q1, q2, q3;

    switch(index){
    case 0: //Eslabon
        ui->test1->display(index);
    break;

    case 1: //Herramienta
        //Se leen las posiciones de los motores
        m.getposition();
        m1.getposition();
        m2.getposition();

        maux = m.dxl_present_position;
        m1aux = m1.dxl_present_position;
        m2aux = m2.dxl_present_position;

        //Se convierten a radianes
        q1 = (maux*grad*rad);
        q2 = (m1aux*grad*rad);
        q3 = (m2aux*grad*rad);

        //Entran a la inversa
        Ik1.InverseKinect(q1,q2,q3,Ik1.px,Ik1.py,Ik1.pz);

        //Se imprimen
        ui->pos_x_herramienta->display(Ik1.px);
        ui->pos_y_herramienta->display(Ik1.py);
        ui->pos_z_herramienta->display(Ik1.pz);

        pxcont = Ik1.px;
        pycont = Ik1.py;
        pzcont = Ik1.pz;

        ui->q1_herramienta->display(q1);
        ui->q2_herramienta->display(q2);
        ui->q3_herramienta->display(q3);

        ui->test1->display(index);
    break;

    case 2: //Trayectoria
        ui->test1->display(index);
    break;
    }

    ui->test1->display(index);
}

