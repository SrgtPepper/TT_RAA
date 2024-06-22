#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QTimer>
#include <fstream>

using namespace std;

Kinematic k1,k2;
InverseKinematic Ik1, Ik2;

ax12a m0(0);
ax12a m1(1);
ax12a m2(2);
ax12a m3(3);

int speed = 200;
void updateMotorsPosition(Vector4d pos);
void toObtainGraphs();
Vector4d positions{0,0,0,0};

Vector4d error1{0,0,0,0}, error2{0,0,0,0};
bool writing = false;
Vector4d present_q{0,0,0,0};
bool controlChange = false, control = false;
double t = 0;
QElapsedTimer elapsedTimer;
ofstream logg1("Errores_sin_control.txt");
ofstream logg2("Errores_control.txt");
ofstream logg3("Angulos_sin_control.txt");
ofstream logg4("Angulos_control.txt");

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , workerThread(new QThread(this))
    , worker(new MyWorker())
{
    ui->setupUi(this);

    QPixmap ipn("/home/shiza/Escritorio/ptm/TT_RAA_Proyecto_Final/TT_RAA_Control_motores_respaldo_para_contrastar_errores/logo-ipn-guinda.png");
    QPixmap upiiz("/home/shiza/Escritorio/ptm/TT_RAA_Proyecto_Final/TT_RAA_Control_motores_respaldo_para_contrastar_errores/UPIIZ.png");

    ui->logo_ipn->setPixmap(ipn.scaled(80,120));
    ui->logo_upiiz->setPixmap(upiiz);

    objWorldGL = new WorldGL(ui->animacion);
    objWorldGL->resize(500,500); // el resize coincide con el tamaño del openGLWidget
    loadRobot();

    QTimer *timer = new QTimer(this);
    QTimer *timer1 = new QTimer(this);
    timer->setInterval(1);
    timer1->setInterval(100);

//    MyWorker *workerGraphs = new MyWorker;
//    QThread *threadGraphs = new QThread;
//    workerGraphs->moveToThread(threadGraphs);

    connect(timer, SIGNAL(timeout()),worker,SLOT(doWork()));
    connect(timer, SIGNAL(timeout()),this,SLOT(toObtainGraphs()));
   // connect(timer1, SIGNAL(timeout()),worker,SLOT(toObtainGraphs()));
    connect(worker,SIGNAL(resultsReady(double,double,double,double))
            ,this,SLOT(handleResults(double,double,double,double)));
    connect(workerThread,SIGNAL(started()),timer,SLOT(start()));
    connect(workerThread, SIGNAL(finished()),worker,SLOT(deleteLater()));
    connect(this,SIGNAL(pestanaClicked(int)),worker,SLOT(onTabBarClicked(int)));
    connect(worker,SIGNAL(slidersValues(int,int,int,int)),this,SLOT(handleSlidersValues(int,int,int,int)));
    connect(this,SIGNAL(goHome()),worker,SLOT(goHome()));
    connect(this, SIGNAL(checkboxControlStateChanged(int)),worker,SLOT(onCheckboxControlStateChanged(int)));
    connect(this, SIGNAL(checkboxFileStateChanged(int)),worker,SLOT(writeOnFile(int)));

    m0.initport();
    m1.initport();
    m2.initport();
    m3.initport();

    m0.enabletorque();
    m1.enabletorque();
    m2.enabletorque();
    m3.enabletorque();

    m0.setspeed(speed);
    m1.setspeed(speed);
    m2.setspeed(speed);
    m3.setspeed(speed);

    worker->moveToThread(workerThread);
    workerThread->start();
    emit goHome();
    elapsedTimer.start();

}

MainWindow::~MainWindow()
{
    turnOffAllMotors();
    workerThread->quit();
    workerThread->wait();

    delete ui;
}

void MainWindow::toObtainGraphs()
{

    if(writing){
//        auto start = chrono::high_resolution_clock::now();
        m0.getposition(), m1.getposition(), m2.getposition(), m3.getposition();
        present_q[0] = (m0.dxl_present_position*mtrToGrad-150)*gradToRad;
        present_q[1] = (m1.dxl_present_position*mtrToGrad-60)*gradToRad;
        present_q[2] = (m2.dxl_present_position*mtrToGrad-150)*gradToRad;
        present_q[3] = (m3.dxl_present_position*mtrToGrad-150)*gradToRad;

        if(controlChange){
            t = 0;
            controlChange = false;
        }

        t = elapsedTimer.elapsed()/1000.0;

        if(!control){
            error1 = worker->qGlobal-present_q;
            logg1 << t << " " << error1[0] << " " << error1[1] << " " << error1[2] << " " << error1[3] << endl;

            logg3 << t << " " << present_q[0]/gradToRad << " " << (present_q[1]/gradToRad) - 90 << " " << present_q[2]/gradToRad << " " << present_q[3]/gradToRad << endl; //Para graficar posiciones angulares
        }else{
            error2 = worker->C->desired_pos-present_q;
            logg2 << t << " " << error2[0] << " " << error2[1] << " " << error2[2] << " " << error2[3] << endl;
            logg4 << t << " " << present_q[0]/gradToRad << " " << (present_q[1]/gradToRad) - 90 << " " << present_q[2]/gradToRad << " " << present_q[3]/gradToRad << endl; //Para graficar posiciones angulares
        }
//        auto end = chrono::high_resolution_clock::now();
//        chrono::duration<double> duration = end-start;
//        cout << "Tiempo de ejecucion: " << duration.count() << " segundos" << endl;
    }

}

void updateMotorsPosition(Vector4d pos)
{
    int qMotor[4];
    qMotor[0] = round((pos[0]+150)/mtrToGrad);
    qMotor[1] = round((pos[1]+60)/mtrToGrad);
    qMotor[2] = round((pos[2]+150)/mtrToGrad);
    qMotor[3] = round((pos[3]+150)/mtrToGrad);

    m0.setposition(qMotor[0]);
    m1.setposition(qMotor[1]);
    m2.setposition(qMotor[2]);
    m3.setposition(qMotor[3]);

}

void MainWindow::turnOffAllMotors()
{
    m0.turnoff();
    m1.turnoff();
    m2.turnoff();
    m3.turnoff();

    logg1.close();
    logg2.close();
    //cout << "Mototes apagados" << endl;
}



void MainWindow::loadRobot()
{
    QString fileName("/home/shiza/Escritorio/ptm/TT_RAA_Proyecto_Final/TT_RAA_Control_motores_respaldo_para_contrastar_errores/momosbot_chidisimo_sin_comas.rbt");

    QFile file(fileName);
    if (file.open(QIODevice::ReadOnly)){
        newRobot = new Robot;
            while (!file.atEnd()) {
                QString line = file.readLine();
                QTextStream lineIn(&line);
                QString aux;
                coordVector cVectorAux;
                triFace triFaceAux;
                float terminatorAux;
                if (line.contains("Body")) {
                    newBody = new Body;
                    lineIn >> aux >> newBody->bodyName >> aux;
                    while (!line.contains("}")){
                        line = file.readLine();
                        QTextStream lineIn(&line);
                        if (line.contains("Transform")){
                            lineIn >> aux >> newBody->transform[0]>>newBody->transform[1]>>newBody->transform[2];
                        }
                        if (line.contains("Rotation")){
                            lineIn >> aux >> newBody->rotation[0]>>newBody->rotation[1]>>newBody->rotation[2]>>newBody->rotation[3];
                        }
                        if (line.contains("Center")){
                            lineIn >> aux >> newBody->center[0]>>newBody->center[1]>>newBody->center[2];
                        }
                        if (line.contains("points")){
                            line = file.readLine();
                            while (!line.contains("]")){
                                QTextStream lineIn(&line);
                                lineIn >> cVectorAux.x >> cVectorAux.y >> cVectorAux.z;
                                newBody->points.push_back(cVectorAux);
                                line = file.readLine();
                            }
                        }
                        if (line.contains("faceIndex")){
                            line = file.readLine();
                            while (!line.contains("]")){
                                QTextStream lineIn(&line);
                                lineIn >> triFaceAux.c1 >> triFaceAux.c2 >> triFaceAux.c3 >> terminatorAux;
                                // If there is a quad make two triangles instead
                                if (terminatorAux<0){
                                    newBody->indexTriFaces.push_back(triFaceAux);
                                }
                                else {
                                    newBody->indexTriFaces.push_back(triFaceAux);
                                    triFaceAux.c2 = triFaceAux.c3;
                                    triFaceAux.c3 = terminatorAux;
                                    newBody->indexTriFaces.push_back(triFaceAux);
                                }
                                line = file.readLine();
                            }
                        }
                    }
                    // Create a random color for each body
                    newBody->color.r = rand() % 155 + 100;
                    newBody->color.g = rand() % 155;
                    newBody->color.b = rand() % 50;
                    newBody->adjustPoints();
                    // Calculate face normals
                    newBody->calculateNormals();
                    newBody->createFaces();
//                    newBody->buildPQPmodel();
                    newRobot->appendBody(*newBody);
                }
                if (line.contains("Joint")) {
                    newJoint = new Joint;
                    lineIn >> aux >> newJoint->jointName >> aux;
                    while (!line.contains("}")) {
                        line = file.readLine();
                        QTextStream lineIn(&line);
                        if (line.contains("jointType")){
                            lineIn >> aux >> newJoint->type;

                        }
                        if (line.contains("jointFather")){
                            lineIn >> aux >> newJoint->fatherString;
                        }
                        if (line.contains("jointChild")){
                            lineIn >> aux;
                            while (!lineIn.atEnd()){
                            lineIn >> aux;
                            newJoint->childString.push_back(aux);
                            }
                        }
                        if (line.contains("directorAxis")){
                            lineIn >> aux >> newJoint->jointAxis.x >> newJoint->jointAxis.y >> newJoint->jointAxis.z;
                        }
                        if (line.contains("limits")){
                            lineIn >> aux >> newJoint->limits.min >> newJoint->limits.max;
                        }
                        if (line.contains("value")){
                            lineIn >> aux >> newJoint->value;
                        }
                    }
                    newRobot->appendJoint(*newJoint);
                }
            }
    }
    file.close();
    newRobot->getJointLimits();
    objWorldGL->appendRobot(*newRobot);
    //int n= newRobot->jointVector.size() -1;
    vector <int> lMax = newRobot->jointLimitsMax;
    vector <int> lMin = newRobot->jointLimitsMin;
    //setSlidersLimits(lMax,lMin);
}

void MainWindow::setRobotConfig(double q1, double q2, double q3, double q4)
{
    if (!objWorldGL->robotVector.empty()){
        vector <float> config;
        config.push_back(q1);
        config.push_back(q2);
        config.push_back(q3);
        config.push_back(q4);

        objWorldGL->setRobotConfig(config,0);
        objWorldGL->updateGL();

    }
}

void MainWindow::handleResults(double q1, double q2, double q3, double q4)
{
    if(!worker->K1->hit){
        control = worker->control;
        controlChange = worker->controlChange;
        setRobotConfig(q1,-1*q2,-1*q3,q4);
        positions << q1, q2, q3, q4;
        updateMotorsPosition(positions);
        ui->q1->display(q1);
        ui->q2->display(q2-90);
        ui->q3->display(q3);
        ui->pos_x->display(worker->coord[0]);
        ui->pos_y->display(worker->coord[1]);
        ui->pos_z->display(worker->coord[2]);
        ui->veslabon1->display(worker->D->qp[0]);
        ui->veslabon2->display(worker->D->qp[1]);
        ui->veslabon3->display(worker->D->qp[2]);
        ui->aeslabon1->display(worker->D->qpp[0]);
        ui->aeslabon2->display(worker->D->qpp[1]);
        ui->aeslabon3->display(worker->D->qpp[2]);
    }
}

void MainWindow::handleSlidersValues(int s1, int s2, int s3, int s4)
{
    ui->eslabon1->setValue(s1);
    ui->eslabon2->setValue(s2);
    ui->eslabon3->setValue(s3);
    ui->eslabon4->setValue(s4);
}



void MainWindow::on_xplus_clicked()
{
    worker->coord[0] += 1;
}


void MainWindow::on_xminus_clicked()
{
    worker->coord[0] -= 1;
}


void MainWindow::on_yplus_clicked()
{
    worker->coord[1] += 1;
}


void MainWindow::on_yminus_clicked()
{
    worker->coord[1] -= 1;
}


void MainWindow::on_zplus_clicked()
{
    worker->coord[2] += 1;
}


void MainWindow::on_zminus_clicked()
{
    worker->coord[2] -= 1;
}


void MainWindow::on_home_clicked()
{
    emit goHome();
}


void MainWindow::on_eslabon1_valueChanged(int value)
{
    worker->K1->q1 = value/worker->a;
}


void MainWindow::on_eslabon2_valueChanged(int value)
{
    worker->K1->q2 = value/worker->a;
}


void MainWindow::on_eslabon3_valueChanged(int value)
{
    worker->K1->q3 = value/worker->a;
}


void MainWindow::on_eslabon4_valueChanged(int value)
{
    worker->K1->q4 = value/worker->a;
}


void MainWindow::on_checkBox_stateChanged(int arg1)
{
  emit checkboxControlStateChanged(arg1);
}


void MainWindow::on_pestanas_tabBarClicked(int index)
{
    emit pestanaClicked(index);
}


void MainWindow::on_checkBox_2_stateChanged(int arg1)
{
    //emit checkboxFileStateChanged(arg1);

    if(arg1==2){
        writing = true;
    }else{
        writing = false;
    }
}


void MainWindow::on_testModeButton_clicked()
{
    worker->coord[0] = ui->xCoord->value();
    worker->coord[1] = ui->yCoord->value();
    worker->coord[2] = ui->zCoord->value();
}

