#ifndef KINEMATIC_H
#define KINEMATIC_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#define _USE_MATH_DEFINES
#include <cmath>
#include "ax12a.h"


class Kinematic
{
public:

    double q1;
    double q2;
    double q3;
    double q4;

    double l1 = 17.22 + 15.155;
    double l2 = 16.022;
    double l3 = 13.457;
    double l4 = 3;

    double px,py,pz;

    double h = 48;
    int r = 12;
    bool hit = false;

    Kinematic();
    ~Kinematic();

    void DirecKinect(double q1,double q2,double q3, double &px, double &py, double &pz);

    void hitbox(double px, double py, double pz, bool &hit);

private:
};

#endif // KINEMATIC_H
