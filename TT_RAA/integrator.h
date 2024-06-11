#ifndef INTEGRATOR_H
#define INTEGRATOR_H
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <QtGlobal>

using namespace Eigen;
typedef qint64 qint64;

class Integrator
{
public:
    Integrator();
    ~Integrator();

    double h = 0.001;                   // paso de integracion

    Vector4d aux = Vector4d::Zero();    // Vector para acumular resultados

    void Integral_Euler(Vector4d in, Vector4d &out);
    void setIntegrationStep(double h);
};

#endif // INTEGRATOR_H
