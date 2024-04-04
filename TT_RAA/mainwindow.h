#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

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

private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
