#ifndef DINAMIC_H
#define DINAMIC_H
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>


using namespace Eigen;

class Dinamic
{
public:

    // Variables artculares
    double q1   = 0;
    double q2   = 0;
    double q3   = 0;
    double q4   = 0;
    double q1p  = 0;
    double q2p  = 0;
    double q3p  = 0;
    double q4p  = 0;
    double q1pp = 0;
    double q2pp = 0;
    double q3pp = 0;
    double q4pp = 0;
    
    // Matrices y vectores para la dinamica
    Matrix4d D = Matrix4d::Zero(), C = Matrix4d::Zero();
    Vector4d Phi = Vector4d::Zero(), Tau = Vector4d::Zero();
    Vector4d q = Vector4d::Zero(), qp = Vector4d::Zero(), qpp = Vector4d::Zero();

    // Declaración de los elementos de las matrices D, C y Phi

    double D11, D12, D13, D14, D21, D22, D23, D24, D31, D32, D33, D34, D41, D42, D43, D44;
    double C11, C12, C13, C14, C21, C22, C23, C24, C31, C32, C33, C34, C41, C42, C43, C44;
    double Phi1, Phi2, Phi3, Phi4;

    // double longitudes[4];
    double l1 ;
    double l2 ;
    double l3 ;
    double l4 ;

    // double longitudes_cm[3];
    double lc2 ;
    double lc3 ;
    double lc4 ;
    
    // double masas[4];
    double m1 ;
    double m2 ;
    double m3 ;
    double m4 ;

    // Valores de los tensores de inercia
    double I1xx = 8.3136e-04, I1xy = 1.4790e-07, I1xz = 3.9469e-06;
    double I1yx = 1.4790e-07, I1yy = 4.5031e-04, I1yz = 5.1323e-06;
    double I1zx = 3.9469e-06, I1zy = 5.1323e-06, I1zz = 5.8475e-04;
    
    double I2xx =  6.0422e-04, I2xy =      1e-11, I2xz = -3.4420e-06;
    double I2yx =       1e-11, I2yy = 2.4434e-03, I2yz =           0;
    double I2zx = -3.4420e-06, I2zy =          0, I2zz =  2.7347e-03;  

    double I3xx =  4.2234e-04, I3xy = -2.5490e-05, I3xz = -1.0580e-04;
    double I3yx = -2.5490e-05, I3yy =  5.9218e-04, I3yz =  1.5662e-05;
    double I3zx = -1.0580e-04, I3zy =  1.5662e-05, I3zz =  3.8552e-04;

    double I4xx = 1.9334e-04, I4xy =          0, I4xz =          0;
    double I4yx =          0, I4yy = 1.9353e-04, I4yz =          0;
    double I4zx =          0, I4zy =          0, I4zz = 2.3538e-04;


    Dinamic();
    ~Dinamic();

    void Dynamic_qpp(Vector4d Tau,Vector4d q, Vector4d qp, Vector4d &qpp);

    // Matrices de ganancias Kp y Kd para el controlador PD+G

    Matrix4d Kp{
        {100,   0,   0,   0},
        {  0, 100,   0,   0},
        {  0,   0, 100,   0},
        {  0,   0,   0, 100}

    };

    Matrix4d Kd{
        { 10,   0,   0,   0},
        {  0,  10,   0,   0},
        {  0,   0,  10,   0},
        {  0,   0,   0,  10}

    };

    // Vectores para calcular los errores

    Vector4d desired_pos, pos_error;
    Vector4d desired_speed, speed_error;

    void control_PD_G(Vector4d desired_pos, Vector4d q, Vector4d desired_speed, Vector4d qp, Vector4d Phi, Vector4d &Tau);
};

#endif // DINAMIC_H
