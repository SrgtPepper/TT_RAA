#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "opt1.h"
#include "opt2.h"
#include "opt3.h"
#include "Kinematic.h"
#include "InverseKinematic.h"
#include "Dinamic.h"
#define PI 3.141594

Kinematic k1;
InverseKinematic Invk1;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

}

MainWindow::~MainWindow()
{
    delete ui;
}


int contadorx = 0;
int contadory = 0;
int contadorz = 0;

////Comportamiento de los btns X
void MainWindow::on_pushButton_clicked()
{
    contadorx++;
    Invk1.InverseKinect(contadorx,b,c,Invk1.q1,Invk1.q2,Invk1.q3);
    ui->lcdNumber->display(Invk1.q1);
    ui->lcdNumber_2->display(Invk1.q2);
    ui->lcdNumber_3->display(Invk1.q3);
    ui->lcdNumber_11->display(contadorx);
}


void MainWindow::on_pushButton_2_clicked()
{
    contadorx--;
    Invk1.InverseKinect(contadorx,b,c,Invk1.q1,Invk1.q2,Invk1.q3);
    ui->lcdNumber->display(Invk1.q1);
    ui->lcdNumber_2->display(Invk1.q2);
    ui->lcdNumber_3->display(Invk1.q3);
    ui->lcdNumber_11->display(contadorx);
}


////Comportamiento de los btns Y
void MainWindow::on_pushButton_3_clicked()
{
    contadory++;
    Invk1.InverseKinect(a,contadory,c,Invk1.q1,Invk1.q2,Invk1.q3);
    ui->lcdNumber->display(Invk1.q1);
    ui->lcdNumber_2->display(Invk1.q2);
    ui->lcdNumber_3->display(Invk1.q3);
    ui->lcdNumber_12->display(contadory);
}


void MainWindow::on_pushButton_4_clicked()
{
    contadory--;
    Invk1.InverseKinect(a,contadory,c,Invk1.q1,Invk1.q2,Invk1.q3);
    ui->lcdNumber->display(Invk1.q1);
    ui->lcdNumber_2->display(Invk1.q2);
    ui->lcdNumber_3->display(Invk1.q3);
    ui->lcdNumber_12->display(contadory);
}

////Comportamiento de los btns Z

void MainWindow::on_pushButton_5_clicked()
{
    contadorz++;
    Invk1.InverseKinect(a,b,contadorz,Invk1.q1,Invk1.q2,Invk1.q3);
    ui->lcdNumber->display(Invk1.q1);
    ui->lcdNumber_2->display(Invk1.q2);
    ui->lcdNumber_3->display(Invk1.q3);
    ui->lcdNumber_13->display(contadorz);
}


void MainWindow::on_pushButton_6_clicked()
{
    contadorz--;
    Invk1.InverseKinect(a,b,contadorz,Invk1.q1,Invk1.q2,Invk1.q3);
    ui->lcdNumber->display(Invk1.q1);
    ui->lcdNumber_2->display(Invk1.q2);
    ui->lcdNumber_3->display(Invk1.q3);
    ui->lcdNumber_13->display(contadorz);
}


//Creacion de pantallas de opciones


void MainWindow::on_actionConfiguracion_de_paso_triggered() //Pantalla de configuracion general
{
    opt1 configpaso;
    configpaso.setModal(true);
    configpaso.exec();
}


void MainWindow::on_actionMostrar_animacion_triggered()     //Pantalla de animaciones
{
    opt3 animation;
    animation.setModal(true);
    animation.exec();
}



void MainWindow::on_horizontalSlider_sliderMoved(int position)  //Sliders
{
    a = position;
    double aux = a*(PI/180);
    k1.DirecKinect(aux,b,c,d,k1.px,k1.py,k1.pz);
    ui->lcdNumber_4->display(k1.px);
    ui->lcdNumber_5->display(k1.py);
    ui->lcdNumber_6->display(k1.pz);
    ui->lcdNumber_8->display(a);
}


void MainWindow::on_horizontalSlider_2_sliderMoved(int position)
{
    b = position;
    double bux = b*(PI/180);
    k1.DirecKinect(a,bux,c,d,k1.px,k1.py,k1.pz);
    ui->lcdNumber_4->display(k1.px);
    ui->lcdNumber_5->display(k1.py);
    ui->lcdNumber_6->display(k1.pz);
    ui->lcdNumber_9->display(b);
}


void MainWindow::on_horizontalSlider_3_sliderMoved(int position)
{
    c=position;
    double cux = c*(PI/180);
    k1.DirecKinect(a,b,cux,d,k1.px,k1.py,k1.pz);
    ui->lcdNumber_4->display(k1.px);
    ui->lcdNumber_5->display(k1.py);
    ui->lcdNumber_6->display(k1.pz);
    ui->lcdNumber_10->display(c);
}


void MainWindow::on_horizontalSlider_4_sliderMoved(int position)
{
    d=position;
    double dux = d*(PI/180);
    ui->lcdNumber_7->display(dux);
}

