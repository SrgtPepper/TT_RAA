#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.setWindowTitle("Brazo robótico modelo San Carlo");
    w.show();
    return a.exec();
}
