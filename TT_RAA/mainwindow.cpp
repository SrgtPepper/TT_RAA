#include "mainwindow.h"
#include "ui_mainwindow.h"

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
int contador = 0;

//Comportamiento del btn X+
void MainWindow::on_pushButton_clicked()
{
    contador++;
    ui->lcdNumber->display(contador);
}

