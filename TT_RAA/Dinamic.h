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

    // longitudes de los eslabones en centimetros
    // double longitudes[4];
    double l1 = 15.155+17.2;
    double l2 = 16.022;
    double l3 = 13.457;
    double l4 = 3;

    // longitudes a lo centros de masa de los eslabones en centimetros
    // double longitudes_cm[3];
    double lc2 = 9.143;
    double lc3 = 6.507;
    double lc4 = 1.697;
    
    // double masas[4];
    // masas de los eslabones en kilogramos
    double m1 = 0.29899;
    double m2 = 0.32392;
    double m3 = 0.22622;
    double m4 = 0.01697;

    // Valores de los tensores de inercia de masa en Kg*cm^2
    double I1xx = 8.3136, I1xy = 1.4790e-03, I1xz = 3.9469e-02;
    double I1yx = 1.4790e-03, I1yy = 4.5031, I1yz = 5.1323e-02;
    double I1zx = 3.9469e-02, I1zy = 5.1323e-02, I1zz = 5.8475;
    
    double I2xx =  6.0422, I2xy =      1e-07, I2xz = -3.4420e-02;
    double I2yx =       1e-07, I2yy = 24.434, I2yz =           0;
    double I2zx = -3.4420e-02, I2zy =          0, I2zz =  27.347;

    double I3xx =  4.2234, I3xy = -2.5490e-01, I3xz = -1.0580;
    double I3yx = -2.5490e-01, I3yy =  5.9218, I3yz =  1.5662e-01;
    double I3zx = -1.0580, I3zy =  1.5662e-01, I3zz =  3.8552;

    double I4xx = 1.9334, I4xy =      0, I4xz =      0;
    double I4yx =      0, I4yy = 1.9353, I4yz =      0;
    double I4zx =      0, I4zy =      0, I4zz = 2.3538;


    Dinamic();
    ~Dinamic();

    void Dynamic_qpp(Vector4d Tau,Vector4d q, Vector4d qp, Vector4d &qpp);


};

#endif // DINAMIC_H
