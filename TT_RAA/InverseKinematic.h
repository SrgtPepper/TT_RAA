#ifndef INVERSEKINEMATIC_H
#define INVERSEKINEMATIC_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>

class InverseKinematic
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

    InverseKinematic();
    ~InverseKinematic();

    void InverseKinect(double px,double py,double pz, double &q1, double &q2, double &q3);
};

#endif // INVERSEKINEMATIC_H
