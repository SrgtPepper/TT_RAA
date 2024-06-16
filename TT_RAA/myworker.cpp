#include "myworker.h"
#include <QThread>
#include <iostream>
#include <iomanip>
#include <QTimer>
#include <cmath>
#define _USE_MATH_DEFINES
#include "integrator.h"
#include <fstream>
#include <chrono>
#include <QElapsedTimer>

using namespace std;

//QElapsedTimer elapsedTimer;


//double t = 0,t1=0;
double tiempo_max = 0;;
//ofstream logg("Errores_implementacion.txt");
//ofstream logg1("Errores_sin_control.txt");
//ofstream logg2("Errores_control.txt");

int i = 0, j = 0,jread=0;


double jointLimit = 5*M_PI/6;
double h = 0.001;
//int speed = 165;
Vector4d home{0,0,0,0}; //vector de posiciones iniciales
int mode_aux;
uint16_t velocidad[4];

MyWorker::MyWorker(QObject *parent) : QObject(parent)
  ,D(new Dinamic), C(new Controller), I_qp(new Integrator)
  , I_q(new Integrator), K1(new Kinematic), K2(new Kinematic)
  , Inv_K(new InverseKinematic), m0(new ax12a(0)), m1(new ax12a(1))
  , m2(new ax12a(2)), m3(new ax12a(3))
{

    inicialize();

}

void MyWorker::inicialize()
{
//    m0->initport();
//    m1->initport();
//    m2->initport();
//    m3->initport();

//    m0->enabletorque();
//    m1->enabletorque();
//    m2->enabletorque();
//    m3->enabletorque();

//    m0->setspeed(speed);
//    m1->setspeed(speed);
//    m2->setspeed(speed);
//    m3->setspeed(speed);

    I_qp->setIntegrationStep(h);
    I_q->setIntegrationStep(h);

    Matrix4d kp{
        {1e6,   0,   0,   0},
        {  0, 5e4,   0,   0},
        {  0,   0, 2e4,  0},
        {  0,   0,   0, 1e2}
    };

    Matrix4d kd{
        {1e5,   0,   0,   0},
        {  0, 8e3,   0,   0},
        {  0,   0, 1e3,   0},
        {  0,   0,   0,  1e1}
    };

    C->set_kp_gains(kp);
    C->set_kd_gains(kd);

 //   elapsedTimer.start();
}

void MyWorker::updateMotorsPosition(Vector4d pos)
{
    qMotor[0] = round((pos[0]/gradToRad+150)/mtrToGrad);
    qMotor[1] = round((pos[1]/gradToRad+60)/mtrToGrad);
    qMotor[2] = round((pos[2]/gradToRad+150)/mtrToGrad);
    qMotor[3] = round((pos[3]/gradToRad+150)/mtrToGrad);

    m0->setposition(qMotor[0]);
    m1->setposition(qMotor[1]);
    m2->setposition(qMotor[2]);
    m3->setposition(qMotor[3]);

}

void MyWorker::getData(Vector4d &pos, Vector4d &vel, Vector4d &accel)
{

    m0->getposition(), m1->getposition(), m2->getposition(), m3->getposition();
    m0->getspeed(), m1->getspeed(), m2->getspeed(), m3->getspeed();

    pos[0] = (m0->dxl_present_position*mtrToGrad-150)*gradToRad;
    pos[1] = (m1->dxl_present_position*mtrToGrad-60)*gradToRad;
    pos[2] = (m2->dxl_present_position*mtrToGrad-150)*gradToRad;
    pos[3] = (m3->dxl_present_position*mtrToGrad-150)*gradToRad;

    velocidad[0] = m0->dxl_present_speed;
    velocidad[1] = m1->dxl_present_speed;
    velocidad[2] = m2->dxl_present_speed;
    velocidad[3] = m3->dxl_present_speed;

    for(i=0;i<4;i++){
        if(velocidad[i]>=1024){
            vel[i] = -1*((velocidad[i]-1024)*rev)/radpsecToRpm;
        }else{
            vel[i] = velocidad[i]*rev/radpsecToRpm;
        }
        accel[i] = vel[i]-old_qp[i];
        old_qp[i] = vel[i];
    }

}

void MyWorker::toObtainGraphs()
{

//    if(writing){
//        auto start = chrono::high_resolution_clock::now();
//        m0->getposition(), m1->getposition(), m2->getposition(), m3->getposition();
//        present_q[0] = (m0->dxl_present_position*mtrToGrad-150)*gradToRad;
//        present_q[1] = (m1->dxl_present_position*mtrToGrad-60)*gradToRad;
//        present_q[2] = (m2->dxl_present_position*mtrToGrad-150)*gradToRad;
//        present_q[3] = (m3->dxl_present_position*mtrToGrad-150)*gradToRad;

//        if(controlChange){
//            t = 0;
//            controlChange = false;
//        }

//        t = elapsedTimer.elapsed()/1000.0;

//        if(!control){
//            error1 = qGlobal-present_q;
//            logg1 << t << " " << error1[0] << " " << error1[1] << " " << error1[2] << " " << error1[3] << endl;
//        }else{
//            error2 = C->desired_pos-present_q;
//            logg2 << t << " " << error2[0] << " " << error2[1] << " " << error2[2] << " " << error2[3] << endl;
//        }
//        auto end = chrono::high_resolution_clock::now();
//        chrono::duration<double> duration = end-start;
//        cout << "Tiempo de ejecucion: " << duration.count() << " segundos" << endl;
//    }

}

void MyWorker::turnOffAllMotors()
{
//    m0->turnoff();
//    m1->turnoff();
//    m2->turnoff();
//    m3->turnoff();

//    logg.close();
//    logg1.close();
//    logg2.close();
    cout << "Mototes apagados" << endl;
}

void MyWorker::on_eslabon1_valueChanged(int value)
{
    K1->q1 = value/a;
    cout << "Valor recibido" << endl;
}


void MyWorker::doWork()
{

    switch(mode){
    case 0:
        q_aux[0] = K1->q1*gradToRad, q_aux[1] = (K1->q2+90)*gradToRad;
        q_aux[2] = K1->q3*gradToRad, q_aux[3] = K1->q4*gradToRad;
        K1->DirecKinect(q_aux[0], q_aux[1], q_aux[2], coord[0], coord[1], coord[2]);

        break;

    case 1:
        Inv_K->InverseKinect(coord[0], coord[1], coord[2], Inv_K->q1, Inv_K->q2, Inv_K->q3);
        q_aux[0] = Inv_K->q1, q_aux[1] = Inv_K->q2, q_aux[2] = Inv_K->q3;
        break;

    case 2: // home
        q_aux = gradToRad*home;

        emit resultsReady(q1,q2,q3,q4);
        mode = mode_aux;
        break;
    }

    K1->hitbox(coord[0], coord[1], coord[2],K1->hit);

    if(!control && !K1->hit){

        for(i = 0; i<4 ; i++){
            if(i!=1){
                if(q_aux[i] < -jointLimit){
                    q_aux[i] = -jointLimit;
                }else if(q_aux[i] > jointLimit){
                    q_aux[i] = jointLimit;
                }
            }else{
                if(q_aux[i] < -(jointLimit-M_PI/2)){
                    q_aux[i] = -(jointLimit-M_PI/2);
                }else if(q_aux[i] > (jointLimit+M_PI/2)){
                    q_aux[i] = jointLimit+M_PI/2;
                }
            }
        }
        qGlobal = q_aux;

    }else if(!K1->hit){

        C->desired_pos = q_aux;

        C->error = C->desired_pos - C->q;

        C->PVG(C->error,C->qp,D->Phi,D->Tau);

        D->Dynamic_qpp(D->Tau,C->q,C->qp,D->qpp);

        I_qp->Integral_Euler(D->qpp,D->qp);
        I_q->Integral_Euler(D->qp,D->q);

        C->qp = D->qp;

        for(i = 0; i<4 ; i++){
            if(i!=1){
                if(D->q[i] < -jointLimit){
                    D->q[i] = -jointLimit;
                }else if(D->q[i] > jointLimit){
                    D->q[i] = jointLimit;
                }
            }else{
                if(D->q[i] < -(jointLimit-M_PI/2)){
                    D->q[i] = -(jointLimit-M_PI/2);
                }else if(D->q[i] > (jointLimit+M_PI/2)){
                    D->q[i] = jointLimit+M_PI/2;
                }
            }
        }



      C->q = D->q;
      qGlobal = D->q;

    }

    q1 = qGlobal[0]/gradToRad, q2 = qGlobal[1]/gradToRad, q3 = qGlobal[2]/gradToRad, q4 = qGlobal[3]/gradToRad;



//    if(j>4){
 //       updateMotorsPosition(qGlobal);
        emit resultsReady(q1,q2,q3,q4);
//        j=0;
//    }else{
//        j++;
//    }

    //t++;

}

void MyWorker::onTabBarClicked(int index)
{
    if(index!=2){
        mode = index;
    }else{
        // cuando se pasa el modo test se sigue ejecutando el modo tool
        mode = 1;
    }

    if(mode==0){

        Inv_K->InverseKinect(coord[0], coord[1], coord[2], qGlobal[0], qGlobal[1], qGlobal[2]);
        s1 = round((qGlobal[0]/gradToRad)*a);
        s2 = round((qGlobal[1]/gradToRad-90)*a);
        s3 = round((qGlobal[2]/gradToRad)*a);
        s4 = round((qGlobal[3]/gradToRad)*a);

        emit slidersValues(s1,s2,s3,s4);

     }else if(mode==1){

        K1->DirecKinect(qGlobal[0], qGlobal[1], qGlobal[2], coord[0], coord[1], coord[2]);

        for(i=0;i<3;i++){
            coord[i] = round(coord[i]);
        }

    }
}

void MyWorker::goHome()
{
    mode_aux = mode;
    mode  = 2; // asi el control tambien participiara llevando el robot a home


    K1->DirecKinect(home[0], home[1], home[2], coord[0], coord[1], coord[2]);

    if(mode_aux == 1){
        for(i=0;i<3;i++){
            coord[i] = round(coord[i]);
        }
    }

    s1 = home[0]*a;
    s2 = (home[1]-90)*a;
    s3 = home[2]*a;
    s4 = home[3]*a;

    emit slidersValues(s1,s2,s3,s4);
}

void MyWorker::onCheckboxControlStateChanged(int state)
{
    if(state == 2 ){
        // para que el control inicie donde se quedo el
        // robot y no desde el las posicion inicial
        I_q->aux = q_aux;
        control = true;
        controlChange = true;
    }else{
        control = false;
        controlChange = true;
    }
}

void MyWorker::writeOnFile(int state)
{

//    if(state==2){
//        writing = true;
//    }else{
//        writing = false;
//    }

}

void MyWorker::readSliders(int value1, int value2, int value3, int value4)
{



}
