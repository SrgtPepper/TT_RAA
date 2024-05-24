#include "trajectories.h"

Trajectories::Trajectories() {}
Trajectories::~Trajectories(){}

void Trajectories::circumference(double r, double t, Vector3d &p)
{
    this->x = 20;
    this->y = r*cos(t);
    this->z = r*sin(t);

    p << x, y, z;
    this->p = p;
}

void Trajectories::lemniscata(double a, double t, Vector3d &p)
{
    this->x = 20;
    this->y = (a*cos(t))/(pow(sin(t),2)+1);
    this->z = (a*cos(t)*sin(t))/(pow(sin(t),2)+1);

    p << x, y, z;
    this->p = p;
}
