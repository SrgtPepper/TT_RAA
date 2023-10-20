#include "opt3.h"
#include "ui_opt3.h"

opt3::opt3(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::opt3)
{
    ui->setupUi(this);
}

opt3::~opt3()
{
    delete ui;
}
