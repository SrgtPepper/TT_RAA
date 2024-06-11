#ifndef MYWORKER_H
#define MYWORKER_H

#include <QObject>
#include <QElapsedTimer>
#include "Dinamic.h"
#include "controller.h"
#include "Kinematic.h"
#include "InverseKinematic.h"
#include "integrator.h"
#include "ax12a.h"
#include <cmath>
#define _USE_MATH_DEFINES

const double rev = 0.111; // equivalencia del valor del registro de velocidad a rpm
const double gradToRad = M_PI/180;  // factor de conversion de grados a radianes
const double mtrToGrad = 0.29297;   // equivalencia entre el valor del registro de posicion a grados
const double radpsecToRpm = 30/M_PI;    // factor de conversion de rad/s a rpm

class MyWorker : public QObject
{
    Q_OBJECT

public:
    explicit MyWorker(QObject *parent = nullptr);

    void inicialize();

    void updateMotorsPosition(Vector4d pos);
    void getData(Vector4d &pos, Vector4d &vel, Vector4d &accel);
    bool control = false;       // para activar/desactivar el control
    bool controlChange = false;
    int mode = 0;                 // para cambiar de modo de operacion
    double s1 = 0, s2 = 0, s3 = 0, s4 = 0;         // para enviar la configuracion de los sliders
    double coord[3] = {0,0,0};            // para guardar las coordenadas
    Vector4d qGlobal{0,0,0,0};          // para guardar los posiciones articulares
    Vector4d  q_aux{0,0,0,0};   // Para guardar momentaneamente las posiciones angulares
    Vector4d present_q, present_qp, old_qp{0,0,0,0}, present_qpp;
    double q1 = 0, q2 = 0, q3 = 0, q4 = 0;         // para emitir resultados
    double a = 100.0;           // factor para escalar los valores de los sliders



    Dinamic *D;
    Controller *C;
    Integrator *I_qp, *I_q;
    Kinematic *K1,*K2;
    InverseKinematic *Inv_K;

    ax12a *m0;
    ax12a *m1;
    ax12a *m2;
    ax12a *m3;

    int qMotor[4] = {0,0,0,0};

signals:
    void resultsReady(double q1, double q2, double q3, double q4);
    void slidersValues(int s1, int s2, int s3, int s4);


public slots:
    void doWork();
    void onTabBarClicked(int index);
    void goHome();
    void onCheckboxControlStateChanged(int state);
    void writeOnFile(int state);
    void readSliders(int value1, int value2, int value3, int value4);
    void toObtainGraphs();
    void turnOffAllMotors();
    void on_eslabon1_valueChanged(int value);
};

#endif // MYWORKER_H
