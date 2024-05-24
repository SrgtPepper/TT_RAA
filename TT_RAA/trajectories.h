#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H
#include <eigen3/Eigen/Core>
#include <math.h>

using namespace Eigen;

class Trajectories
{
public:
    Trajectories();
    ~Trajectories();

    double t = 0;

    double x,y,z;
    Vector3d p = Vector3d::Zero();

    // Parametros para las curvas
    double r;   // circunferencia
    double a;   // lemniscata


    void circumference(double r, double t, Vector3d &p);
    void lemniscata(double a, double t, Vector3d &p);

};

#endif // TRAJECTORIES_H
