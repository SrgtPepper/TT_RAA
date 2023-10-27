#ifndef KINEMATIC_H
#define KINEMATIC_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>

//typedef struct {        //Se definen las posiciones angulares de los motores y las distancias entre eslabones
//}QS;


class Kinematic
{
public:

    double q1;
    double q2;
    double q3;
    double q4;
    double l1 = 10;
    double l2 = 10;
    double l3 = 10;
    double l4 = 10;
    double px,py,pz;

    Kinematic();
    ~Kinematic();

    void DirecKinect(double q1,double q2,double q3,double q4, double &px, double &py, double &pz);

private:
};

#endif // KINEMATIC_H
