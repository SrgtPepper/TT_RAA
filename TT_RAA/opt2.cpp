#include "opt2.h"
#include "ui_opt2.h"

opt2::opt2(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::opt2)
{
    ui->setupUi(this);
}

opt2::~opt2()
{
    delete ui;
}
