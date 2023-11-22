#ifndef INTEGRATOR_H
#define INTEGRATOR_H
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>

/* Header class for integrating for Euler's method */

using namespace Eigen;



class Integrator
{
public:

    double h = 0.1;  // Paso de integracion
    Vector4d aux = Vector4d::Zero();

    Integrator();
    ~Integrator();

    Vector4d integral(Vector4d value,Vector4d &aux);
};

#endif // INTEGRATOR_H
