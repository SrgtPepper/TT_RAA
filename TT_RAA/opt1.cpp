#include "opt1.h"
#include "ui_opt1.h"

opt1::opt1(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::opt1)
{
    ui->setupUi(this);
}

opt1::~opt1()
{
    delete ui;
}
