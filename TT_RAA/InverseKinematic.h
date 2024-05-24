#ifndef INVERSEKINEMATIC_H
#define INVERSEKINEMATIC_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#define _USE_MATH_DEFINES
#include <cmath>

class InverseKinematic
{
public:

    double q1;
    double q2;
    double q3;
    double q4;

    double l1 = 17.22 + 15.155;
    double l2 = 16.022;
    double l3 = 13.457 + 3;

    double px,py,pz;

    double h = 48;
    int r = 12;
    bool hit = false;
    int ec = 1;

    InverseKinematic();
    ~InverseKinematic();

    void InverseKinect(double px,double py,double pz, double &q1, double &q2, double &q3);

    void hitbox(double px, double py, double pz, bool &hit);
};

#endif // INVERSEKINEMATIC_H
