#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <eigen3/Eigen/Dense>
#include <math.h>

using namespace Eigen;

class Controller
{
public:
    Controller();
    ~Controller();

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

    Vector4d desired_pos = Vector4d::Zero();
    Vector4d error = Vector4d::Zero();
    Vector4d q = Vector4d::Zero();
    Vector4d qp = Vector4d::Zero();
   // Vector4d aux = Vector4d::Zero();



    void PVG(Vector4d error, Vector4d qp, Vector4d Phi, Vector4d &Tau);
    void set_kp_gains(Matrix4d kp_new);
    void set_kd_gains(Matrix4d kd_new);

};

#endif // CONTROLLER_H
