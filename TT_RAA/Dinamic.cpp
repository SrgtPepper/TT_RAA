#include "Dinamic.h"
#include "Integrator.h"
#include <eigen3/Eigen/Dense>
#include <math.h>
const double g = 9.81;

using namespace Eigen;
using namespace std;


Dinamic::Dinamic()
{

}

Dinamic::~Dinamic()
{

}

void Dinamic::Dynamic_qpp(Vector4d Tau,Vector4d q, Vector4d qp, Vector4d &qpp)
{

    q1 = q(1), q2 = q(2), q3 = q(3), q4 = q(4);
    q1p = qp(1), q2p = qp(2), q3p = qp(3), q4p = qp(4);

    // Asignacion de las ecuaciones de cada elemento de Inercia

    D11 = I3xx/2 + (3*I3yy)/2 + I1zz + I2zz + exp2(l1)*m1 + exp2(l1)*m2 + exp2(l2)*m1 + exp2(l1)*m3 + exp2(l2)*m2 + exp2(l3)*m1 + exp2(l1)*m4 + exp2(l2)*m3 + exp2(l3)*m2 + exp2(l4)*m1 + exp2(l2)*m4 + exp2(l3)*m3 + exp2(l4)*m2 + exp2(l3)*m4 + exp2(l4)*m3 + exp2(l4)*m4 - (I3xx*cos(2*q4))/2 + (I3yy*cos(2*q4))/2 + (I3xy*sin(2*q4))/2 + (I3yx*sin(2*q4))/2 + 2*l1*l3*m1*cos(q2 + q3) + 2*l1*l3*m2*cos(q2 + q3) + 2*l1*l3*m3*cos(q2 + q3) + 2*l1*l3*m4*cos(q2 + q3) + 2*l1*l4*m1*sin(q2 + q3) + 2*l1*l4*m2*sin(q2 + q3) + 2*l1*l4*m3*sin(q2 + q3) + 2*l1*l4*m4*sin(q2 + q3) + 2*l2*l4*m1*cos(q3) + 2*l2*l4*m2*cos(q3) + 2*l2*l4*m3*cos(q3) + 2*l2*l4*m4*cos(q3) + 2*l1*l2*m1*sin(q2) + 2*l1*l2*m2*sin(q2) + 2*l1*l2*m3*sin(q2) + 2*l1*l2*m4*sin(q2) - 2*l2*l3*m1*sin(q3) - 2*l2*l3*m2*sin(q3) - 2*l2*l3*m3*sin(q3) - 2*l2*l3*m4*sin(q3);
    D12 = I3xx/2 + (3*I3yy)/2 + I2zz + exp2(l2)*m2 + exp2(l2)*m3 + exp2(l3)*m2 + exp2(l2)*m4 + exp2(l3)*m3 + exp2(l4)*m2 + exp2(l3)*m4 + exp2(l4)*m3 + exp2(l4)*m4 - (I3xx*cos(2*q4))/2 + (I3yy*cos(2*q4))/2 + (I3xy*sin(2*q4))/2 + (I3yx*sin(2*q4))/2 + l1*l3*m2*cos(q2 + q3) + l1*l3*m3*cos(q2 + q3) + l1*l3*m4*cos(q2 + q3) + l1*l4*m2*sin(q2 + q3) + l1*l4*m3*sin(q2 + q3) + l1*l4*m4*sin(q2 + q3) + 2*l2*l4*m2*cos(q3) + 2*l2*l4*m3*cos(q3) + 2*l2*l4*m4*cos(q3) + l1*l2*m2*sin(q2) + l1*l2*m3*sin(q2) + l1*l2*m4*sin(q2) - 2*l2*l3*m2*sin(q3) - 2*l2*l3*m3*sin(q3) - 2*l2*l3*m4*sin(q3);
    D13 = I3yz + I3yz*cos(q4) + I3xz*sin(q4);
    D14 = I3yz*cos(q4) + I3xz*sin(q4);
    D21 = I3xx/2 + (3*I3yy)/2 + I2zz + exp2(l2)*m2 + exp2(l2)*m3 + exp2(l3)*m2 + exp2(l2)*m4 + exp2(l3)*m3 + exp2(l4)*m2 + exp2(l3)*m4 + exp2(l4)*m3 + exp2(l4)*m4 - (I3xx*cos(2*q4))/2 + (I3yy*cos(2*q4))/2 + (I3xy*sin(2*q4))/2 + (I3yx*sin(2*q4))/2 + l1*l3*m2*cos(q2 + q3) + l1*l3*m3*cos(q2 + q3) + l1*l3*m4*cos(q2 + q3) + l1*l4*m2*sin(q2 + q3) + l1*l4*m3*sin(q2 + q3) + l1*l4*m4*sin(q2 + q3) + 2*l2*l4*m2*cos(q3) + 2*l2*l4*m3*cos(q3) + 2*l2*l4*m4*cos(q3) + l1*l2*m2*sin(q2) + l1*l2*m3*sin(q2) + l1*l2*m4*sin(q2) - 2*l2*l3*m2*sin(q3) - 2*l2*l3*m3*sin(q3) - 2*l2*l3*m4*sin(q3);
    D22 = I3xx/2 + (3*I3yy)/2 + I2zz + exp2(l2)*m2 + exp2(l2)*m3 + exp2(l3)*m2 + exp2(l2)*m4 + exp2(l3)*m3 + exp2(l4)*m2 + exp2(l3)*m4 + exp2(l4)*m3 + exp2(l4)*m4 - (I3xx*cos(2*q4))/2 + (I3yy*cos(2*q4))/2 + (I3xy*sin(2*q4))/2 + (I3yx*sin(2*q4))/2 + 2*l2*l4*m2*cos(q3) + 2*l2*l4*m3*cos(q3) + 2*l2*l4*m4*cos(q3) - 2*l2*l3*m2*sin(q3) - 2*l2*l3*m3*sin(q3) - 2*l2*l3*m4*sin(q3);
    D23 = I3yz + I3yz*cos(q4) + I3xz*sin(q4);
    D24 = I3yz*cos(q4) + I3xz*sin(q4);
    D31 = I3zy + I3zy*cos(q4) + I3zx*sin(q4);
    D32 = I3zy + I3zy*cos(q4) + I3zx*sin(q4);
    D33 = 2*I3zz + exp2(l3)*m3 + exp2(l3)*m4;
    D34 = I3zz;
    D41 = I3zy*cos(q4) + I3zx*sin(q4);
    D42 = I3zy*cos(q4) + I3zx*sin(q4);
    D43 = I3zz;
    D44 = I3zz;


    // Matriz de inercia
    D << D11, D12, D13, D14,
         D21, D22, D23, D24,
         D31, D32, D33, D34,
         D41, D42, D43, D44;


    // Asignacion de las ecuaciones de cada elemento de la matriz de Coriolis

    C11 = 0;
    C12 = l1*l4*m1*cos(q2 + q3) + 2*l1*l4*m2*cos(q2 + q3) + 2*l1*l4*m3*cos(q2 + q3) + 2*l1*l4*m4*cos(q2 + q3) - l1*l3*m1*sin(q2 + q3) - 2*l1*l3*m2*sin(q2 + q3) - 2*l1*l3*m3*sin(q2 + q3) - 2*l1*l3*m4*sin(q2 + q3) + l1*l2*m1*cos(q2) + 2*l1*l2*m2*cos(q2) + 2*l1*l2*m3*cos(q2) + 2*l1*l2*m4*cos(q2);
    C13 = l1*l4*m1*cos(q2 + q3) + (3*l1*l4*m2*cos(q2 + q3))/2 + (3*l1*l4*m3*cos(q2 + q3))/2 + (3*l1*l4*m4*cos(q2 + q3))/2 - l1*l3*m1*sin(q2 + q3) - (3*l1*l3*m2*sin(q2 + q3))/2 - (3*l1*l3*m3*sin(q2 + q3))/2 - (3*l1*l3*m4*sin(q2 + q3))/2 - l2*l3*m1*cos(q3) - 2*l2*l3*m2*cos(q3) - 2*l2*l3*m3*cos(q3) - 2*l2*l3*m4*cos(q3) - l2*l4*m1*sin(q3) - 2*l2*l4*m2*sin(q3) - 2*l2*l4*m3*sin(q3) - 2*l2*l4*m4*sin(q3);
    C14 = (3*I3xz*cos(q4))/2 - (3*I3yz*sin(q4))/2 + I3xy*cos(2*q4) + I3yx*cos(2*q4) + I3xx*sin(2*q4) - I3yy*sin(2*q4);
    C21 = l1*l3*m1*sin(q2 + q3) - l1*l4*m2*cos(q2 + q3) - l1*l4*m3*cos(q2 + q3) - l1*l4*m4*cos(q2 + q3) - l1*l4*m1*cos(q2 + q3) + l1*l3*m2*sin(q2 + q3) + l1*l3*m3*sin(q2 + q3) + l1*l3*m4*sin(q2 + q3) - l1*l2*m1*cos(q2) - l1*l2*m2*cos(q2) - l1*l2*m3*cos(q2) - l1*l2*m4*cos(q2);
    C22 = 0;
    C23 = (l1*l4*m2*cos(q2 + q3))/2 + (l1*l4*m3*cos(q2 + q3))/2 + (l1*l4*m4*cos(q2 + q3))/2 - (l1*l3*m2*sin(q2 + q3))/2 - (l1*l3*m3*sin(q2 + q3))/2 - (l1*l3*m4*sin(q2 + q3))/2 - 2*l2*l3*m2*cos(q3) - 2*l2*l3*m3*cos(q3) - 2*l2*l3*m4*cos(q3) - 2*l2*l4*m2*sin(q3) - 2*l2*l4*m3*sin(q3) - 2*l2*l4*m4*sin(q3);
    C24 = (3*I3xz*cos(q4))/2 - (3*I3yz*sin(q4))/2 + I3xy*cos(2*q4) + I3yx*cos(2*q4) + I3xx*sin(2*q4) - I3yy*sin(2*q4);
    C31 = l1*l3*m1*sin(q2 + q3) - l1*l4*m2*cos(q2 + q3) - l1*l4*m3*cos(q2 + q3) - l1*l4*m4*cos(q2 + q3) - l1*l4*m1*cos(q2 + q3) + l1*l3*m2*sin(q2 + q3) + l1*l3*m3*sin(q2 + q3) + l1*l3*m4*sin(q2 + q3) + l2*l3*m1*cos(q3) + l2*l3*m2*cos(q3) + l2*l3*m3*cos(q3) + l2*l3*m4*cos(q3) + l2*l4*m1*sin(q3) + l2*l4*m2*sin(q3) + l2*l4*m3*sin(q3) + l2*l4*m4*sin(q3);
    C32 = (l1*l3*m2*sin(q2 + q3))/2 - (l1*l4*m3*cos(q2 + q3))/2 - (l1*l4*m4*cos(q2 + q3))/2 - (l1*l4*m2*cos(q2 + q3))/2 + (l1*l3*m3*sin(q2 + q3))/2 + (l1*l3*m4*sin(q2 + q3))/2 + 2*l2*l3*m2*cos(q3) + 2*l2*l3*m3*cos(q3) + 2*l2*l3*m4*cos(q3) + 2*l2*l4*m2*sin(q3) + 2*l2*l4*m3*sin(q3) + 2*l2*l4*m4*sin(q3);
    C33 = 0;
    C34 = I3zx*cos(q4) - I3zy*sin(q4);
    C41 = (I3yy*sin(2*q4))/2 - (I3yx*cos(2*q4))/2 - (I3xx*sin(2*q4))/2 - (I3xy*cos(2*q4))/2;
    C42 = I3yy*sin(2*q4) - I3yx*cos(2*q4) - I3xx*sin(2*q4) - I3xy*cos(2*q4);
    C43 = I3yz*sin(q4) - I3xz*cos(q4);
    C44 = I3zx*cos(q4) - I3xz*cos(q4) + I3yz*sin(q4) - I3zy*sin(q4);


    // Matriz de Coriolis

    C << C11, C12, C13, C14,
         C21, C22, C23, C24,
         C31, C32, C33, C34,
         C41, C42, C43, C44;     
         
    
    // Elementos del vector de pares gravitacionales

    Phi1 = 0;
    Phi2 = g*m4*(l3*cos(q2 + q3) + l2*cos(q2) + lc4*cos(q2 + q3 + q4)) + g*m3*(lc3*cos(q2 + q3) + l2*cos(q2)) + g*lc2*m2*cos(q2);
    Phi3 = g*m4*(l3*cos(q2 + q3) + lc4*cos(q2 + q3 + q4)) + g*lc3*m3*cos(q2 + q3);
    Phi4 = g*lc4*m4*cos(q2 + q3 + q4);
           
    // Vector de pares gravitacionales

    Phi << Phi1, Phi2, Phi3, Phi4;       

    // Ecuación de la dinamica con las aceleraciones despejadas

    qpp = (D.inverse())*(Tau-C*qp-Phi);

    // Se integran las aceleraciones y velocidades
    /*
    Integrator Accel_Integrator, Speed_Integrator;
    qp = Accel_Integrator.integral(qpp);
    q  = Speed_Integrator.integral(qp);
    */
  /*// Parametros para integrar por el metodo de Euler
    double h = 0.0001;
    //double n = 10000;
    double aux_previous_value1[4] = {0,0,0,0};  // Para guardar los valores previos al integrar las aceleraciones
    double aux_previous_value2[4] = {0,0,0,0};  // para guardar los calores previos al integrar las velocidades
    double results_qp[4] = {0,0,0,0};
    double results_q[4] = {0,0,0,0};
    int i, j, k;

    //for(i = 0; i<n; i++){

        for(j=0; j<4;j++) {
            results_qp[j] = aux_previous_value1[j]+h*qpp(j);
            aux_previous_value1[j] = results_qp[j];
         }

        for(k=0;k<4;k++) qp(k) = results_qp[k];

        for(j=0; j<4;j++) {
            results_q[j] = aux_previous_value2[j]+h*qp(j);
            aux_previous_value1[j] = results_q[j];
        }

        for(k=0;k<4;k++) q(k) = results_q[k];
    //}*/


}

void Dinamic::control_PD_G(Vector4d desired_pos, Vector4d q, Vector4d desired_speed, Vector4d qp, Vector4d Phi, Vector4d &Tau){

    pos_error = desired_pos-q;
    speed_error = desired_speed-qp;

    Tau = Kp*pos_error+Kd*speed_error+Phi;

}
