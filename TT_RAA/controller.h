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

    Vector4d desired_pos, pos_error;
    Vector4d desired_speed, speed_error;

    void PVG(Vector4d desired_pos, Vector4d q, Vector4d qp, Vector4d Phi, Vector4d &Tau);
    void set_kp_gains(Matrix4d kp_new);
    void set_kd_gains(Matrix4d kd_new);

};

#endif // CONTROLLER_H
