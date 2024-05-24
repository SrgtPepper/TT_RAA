#include "Dinamic.h"
#include <eigen3/Eigen/Dense>
#define _USE_MATH_DEFINES
#include <cmath>
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

    D11 = I4xx/2 + I3yy + I4yy/2 + I1zz + I2zz + pow(l1,2)*m1 + pow(l1,2)*m2 + pow(l2,2)*m1 + pow(l1,2)*m3 + pow(l2,2)*m2 + pow(l3,2)*m1 + pow(l1,2)*m4 + pow(l2,2)*m3 + pow(l3,2)*m2 + pow(l4,2)*m1 + pow(l2,2)*m4 + pow(l3,2)*m3 + pow(l4,2)*m2 + pow(l3,2)*m4 + pow(l4,2)*m3 + pow(l4,2)*m4 - (I4xx*cos(2*q4))/2 + (I4yy*cos(2*q4))/2 + 2*l3*l4*m1 + 2*l3*l4*m2 + 2*l3*l4*m3 + 2*l3*l4*m4 + 2*l1*l3*m1*sin(q2 + q3) + 2*l1*l3*m2*sin(q2 + q3) + 2*l1*l4*m1*sin(q2 + q3) + 2*l1*l3*m3*sin(q2 + q3) + 2*l1*l4*m2*sin(q2 + q3) + 2*l1*l3*m4*sin(q2 + q3) + 2*l1*l4*m3*sin(q2 + q3) + 2*l1*l4*m4*sin(q2 + q3) + 2*l2*l3*m1*cos(q3) + 2*l2*l3*m2*cos(q3) + 2*l2*l4*m1*cos(q3) + 2*l2*l3*m3*cos(q3) + 2*l2*l4*m2*cos(q3) + 2*l2*l3*m4*cos(q3) + 2*l2*l4*m3*cos(q3) + 2*l2*l4*m4*cos(q3) + 2*l1*l2*m1*sin(q2) + 2*l1*l2*m2*sin(q2) + 2*l1*l2*m3*sin(q2) + 2*l1*l2*m4*sin(q2);
    D12 = I4xx/2 + I3yy + I4yy/2 + I2zz + pow(l2,2)*m2 + pow(l2,2)*m3 + pow(l3,2)*m2 + pow(l2,2)*m4 + pow(l3,2)*m3 + pow(l4,2)*m2 + pow(l3,2)*m4 + pow(l4,2)*m3 + pow(l4,2)*m4 - (I4xx*cos(2*q4))/2 + (I4yy*cos(2*q4))/2 + 2*l3*l4*m2 + 2*l3*l4*m3 + 2*l3*l4*m4 + l1*l3*m2*sin(q2 + q3) + l1*l3*m3*sin(q2 + q3) + l1*l4*m2*sin(q2 + q3) + l1*l3*m4*sin(q2 + q3) + l1*l4*m3*sin(q2 + q3) + l1*l4*m4*sin(q2 + q3) + 2*l2*l3*m2*cos(q3) + 2*l2*l3*m3*cos(q3) + 2*l2*l4*m2*cos(q3) + 2*l2*l3*m4*cos(q3) + 2*l2*l4*m3*cos(q3) + 2*l2*l4*m4*cos(q3) + l1*l2*m2*sin(q2) + l1*l2*m3*sin(q2) + l1*l2*m4*sin(q2);
    D13 = 0;
    D14 = 0;
    D21 = I4xx/2 + I3yy + I4yy/2 + I2zz + pow(l2,2)*m2 + pow(l2,2)*m3 + pow(l3,2)*m2 + pow(l2,2)*m4 + pow(l3,2)*m3 + pow(l4,2)*m2 + pow(l3,2)*m4 + pow(l4,2)*m3 + pow(l4,2)*m4 - (I4xx*cos(2*q4))/2 + (I4yy*cos(2*q4))/2 + 2*l3*l4*m2 + 2*l3*l4*m3 + 2*l3*l4*m4 + l1*l3*m2*sin(q2 + q3) + l1*l3*m3*sin(q2 + q3) + l1*l4*m2*sin(q2 + q3) + l1*l3*m4*sin(q2 + q3) + l1*l4*m3*sin(q2 + q3) + l1*l4*m4*sin(q2 + q3) + 2*l2*l3*m2*cos(q3) + 2*l2*l3*m3*cos(q3) + 2*l2*l4*m2*cos(q3) + 2*l2*l3*m4*cos(q3) + 2*l2*l4*m3*cos(q3) + 2*l2*l4*m4*cos(q3) + l1*l2*m2*sin(q2) + l1*l2*m3*sin(q2) + l1*l2*m4*sin(q2);
    D22 = I4xx + I3yy + I2zz + pow(l2,2)*m2 + pow(l2,2)*m3 + pow(l3,2)*m2 + pow(l2,2)*m4 + pow(l3,2)*m3 + pow(l4,2)*m2 + pow(l3,2)*m4 + pow(l4,2)*m3 + pow(l4,2)*m4 - I4xx*pow(cos(q4),2) + I4yy*pow(cos(q4),2) + 2*l3*l4*m2 + 2*l3*l4*m3 + 2*l3*l4*m4 + 2*l2*l3*m2*cos(q3) + 2*l2*l3*m3*cos(q3) + 2*l2*l4*m2*cos(q3) + 2*l2*l3*m4*cos(q3) + 2*l2*l4*m3*cos(q3) + 2*l2*l4*m4*cos(q3);
    D23 = 0;
    D24 = 0;
    D31 = 0;
    D32 = 0;
    D33 = I3zz + I4zz;
    D34 = I4zz;
    D41 = 0;
    D42 = 0;
    D43 = I4zz;
    D44 = I4zz;


    // Matriz de inercia
    D << D11, D12, D13, D14,
         D21, D22, D23, D24,
         D31, D32, D33, D34,
         D41, D42, D43, D44;


    // Asignacion de las ecuaciones de cada elemento de la matriz de Coriolis

    C11 = 0;
    C12 = q2p*(l1*l3*m2*cos(q2 + q3) + l1*l3*m3*cos(q2 + q3) + l1*l4*m2*cos(q2 + q3) + l1*l3*m4*cos(q2 + q3) + l1*l4*m3*cos(q2 + q3) + l1*l4*m4*cos(q2 + q3) + l1*l2*m2*cos(q2) + l1*l2*m3*cos(q2) + l1*l2*m4*cos(q2)) + q1p*(l1*l3*m1*cos(q2 + q3) + l1*l3*m2*cos(q2 + q3) + l1*l4*m1*cos(q2 + q3) + l1*l3*m3*cos(q2 + q3) + l1*l4*m2*cos(q2 + q3) + l1*l3*m4*cos(q2 + q3) + l1*l4*m3*cos(q2 + q3) + l1*l4*m4*cos(q2 + q3) + l1*l2*m1*cos(q2) + l1*l2*m2*cos(q2) + l1*l2*m3*cos(q2) + l1*l2*m4*cos(q2));
    C13 = q2p*((l1*l3*m2*cos(q2 + q3))/2 + (l1*l3*m3*cos(q2 + q3))/2 + (l1*l4*m2*cos(q2 + q3))/2 + (l1*l3*m4*cos(q2 + q3))/2 + (l1*l4*m3*cos(q2 + q3))/2 + (l1*l4*m4*cos(q2 + q3))/2 - l2*l3*m2*sin(q3) - l2*l3*m3*sin(q3) - l2*l4*m2*sin(q3) - l2*l3*m4*sin(q3) - l2*l4*m3*sin(q3) - l2*l4*m4*sin(q3)) + q1p*(l1*l3*m1*cos(q2 + q3) + l1*l3*m2*cos(q2 + q3) + l1*l4*m1*cos(q2 + q3) + l1*l3*m3*cos(q2 + q3) + l1*l4*m2*cos(q2 + q3) + l1*l3*m4*cos(q2 + q3) + l1*l4*m3*cos(q2 + q3) + l1*l4*m4*cos(q2 + q3) - l2*l3*m1*sin(q3) - l2*l3*m2*sin(q3) - l2*l4*m1*sin(q3) - l2*l3*m3*sin(q3) - l2*l4*m2*sin(q3) - l2*l3*m4*sin(q3) - l2*l4*m3*sin(q3) - l2*l4*m4*sin(q3));
    C14 = q1p*((I4xx*sin(2*q4))/2 - (I4yy*sin(2*q4))/2) + q2p*((I4xx*sin(2*q4))/2 - (I4yy*sin(2*q4))/2);
    C21 = -q1p*(l1*l3*m1*cos(q2 + q3) + l1*l3*m2*cos(q2 + q3) + l1*l4*m1*cos(q2 + q3) + l1*l3*m3*cos(q2 + q3) + l1*l4*m2*cos(q2 + q3) + l1*l3*m4*cos(q2 + q3) + l1*l4*m3*cos(q2 + q3) + l1*l4*m4*cos(q2 + q3) + l1*l2*m1*cos(q2) + l1*l2*m2*cos(q2) + l1*l2*m3*cos(q2) + l1*l2*m4*cos(q2));
    C22 = 0;
    C23 = q1p*((l1*l3*m2*cos(q2 + q3))/2 + (l1*l3*m3*cos(q2 + q3))/2 + (l1*l4*m2*cos(q2 + q3))/2 + (l1*l3*m4*cos(q2 + q3))/2 + (l1*l4*m3*cos(q2 + q3))/2 + (l1*l4*m4*cos(q2 + q3))/2 - l2*l3*m2*sin(q3) - l2*l3*m3*sin(q3) - l2*l4*m2*sin(q3) - l2*l3*m4*sin(q3) - l2*l4*m3*sin(q3) - l2*l4*m4*sin(q3)) - q2p*(l2*l3*m2*sin(q3) + l2*l3*m3*sin(q3) + l2*l4*m2*sin(q3) + l2*l3*m4*sin(q3) + l2*l4*m3*sin(q3) + l2*l4*m4*sin(q3));
    C24 = q1p*((I4xx*sin(2*q4))/2 - (I4yy*sin(2*q4))/2) + q2p*(I4xx*cos(q4)*sin(q4) - I4yy*cos(q4)*sin(q4));
    C31 = -q1p*(l1*l3*m1*cos(q2 + q3) + l1*l3*m2*cos(q2 + q3) + l1*l4*m1*cos(q2 + q3) + l1*l3*m3*cos(q2 + q3) + l1*l4*m2*cos(q2 + q3) + l1*l3*m4*cos(q2 + q3) + l1*l4*m3*cos(q2 + q3) + l1*l4*m4*cos(q2 + q3) - l2*l3*m1*sin(q3) - l2*l3*m2*sin(q3) - l2*l4*m1*sin(q3) - l2*l3*m3*sin(q3) - l2*l4*m2*sin(q3) - l2*l3*m4*sin(q3) - l2*l4*m3*sin(q3) - l2*l4*m4*sin(q3));
    C32 = q2p*(l2*l3*m2*sin(q3) + l2*l3*m3*sin(q3) + l2*l4*m2*sin(q3) + l2*l3*m4*sin(q3) + l2*l4*m3*sin(q3) + l2*l4*m4*sin(q3)) - q1p*((l1*l3*m2*cos(q2 + q3))/2 + (l1*l3*m3*cos(q2 + q3))/2 + (l1*l4*m2*cos(q2 + q3))/2 + (l1*l3*m4*cos(q2 + q3))/2 + (l1*l4*m3*cos(q2 + q3))/2 + (l1*l4*m4*cos(q2 + q3))/2 - l2*l3*m2*sin(q3) - l2*l3*m3*sin(q3) - l2*l4*m2*sin(q3) - l2*l3*m4*sin(q3) - l2*l4*m3*sin(q3) - l2*l4*m4*sin(q3));
    C33 = 0;
    C34 = 0;
    C41 = -q1p*((I4xx*sin(2*q4))/2 - (I4yy*sin(2*q4))/2);
    C42 = - q1p*((I4xx*sin(2*q4))/2 - (I4yy*sin(2*q4))/2) - q2p*(I4xx*cos(q4)*sin(q4) - I4yy*cos(q4)*sin(q4));
    C43 = 0;
    C44 = 0;


    // Matriz de Coriolis

    C << C11, C12, C13, C14,
         C21, C22, C23, C24,
         C31, C32, C33, C34,
         C41, C42, C43, C44;     
         
    
    // Elementos del vector de pares gravitacionales

    Phi1 = 0;
    Phi2 = g*m4*(cos(q2 + q3)*(l3 + lc4) + l2*cos(q2)) + g*m3*(lc3*cos(q2 + q3) + l2*cos(q2)) + g*lc2*m2*cos(q2);
    Phi3 = g*m4*cos(q2 + q3)*(l3 + lc4) + g*lc3*m3*cos(q2 + q3);
    Phi4 = 0;
           
    // Vector de pares gravitacionales

    Phi << Phi1, Phi2, Phi3, Phi4;       

    // Ecuación de la dinamica con las aceleraciones despejadas

    qpp = (D.inverse())*(Tau-C*qp-Phi);


}


