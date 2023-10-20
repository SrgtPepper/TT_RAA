#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "opt1.h"
#include "opt2.h"
#include "opt3.h"

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

//Comportamiento de los btns X
void MainWindow::on_pushButton_clicked()
{
    contadorx++;
    ui->lcdNumber->display(contadorx);
}


void MainWindow::on_pushButton_2_clicked()
{
    contadorx--;
    ui->lcdNumber->display(contadorx);
}


//Comportamiento de los btns Y
void MainWindow::on_pushButton_3_clicked()
{
    contadory++;
    ui->lcdNumber_2->display(contadory);
}


void MainWindow::on_pushButton_4_clicked()
{
    contadory--;
    ui->lcdNumber_2->display(contadory);
}

//Comportamiento de los btns Z

void MainWindow::on_pushButton_5_clicked()
{
    contadorz++;
    ui->lcdNumber_3->display(contadorz);
}


void MainWindow::on_pushButton_6_clicked()
{
    contadorz--;
    ui->lcdNumber_3->display(contadorz);
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

