#include "Kinematic.h"
#include <eigen3/Eigen/Dense>

using namespace Eigen;
using namespace std;

Matrix4f mht;   //Declaracion de la matriz homogenea total

Kinematic::Kinematic()
{

}


Kinematic::~Kinematic()
{

}



void Kinematic::DirecKinect(double q1,double q2,double q3,double q4, double &px, double &py, double &pz){


    mht << sin(q1)*sin(q4) - cos(q1)*cos(q2)*cos(q4)*sin(q3) - cos(q1)*cos(q3)*cos(q4)*sin(q2), //m(0,0)
           cos(q4)*sin(q1) + cos(q1)*cos(q2)*sin(q3)*sin(q4) + cos(q1)*cos(q3)*sin(q2)*sin(q4), //m(0,1)
           cos(q2 + q3)*cos(q1),    //m(0,2)
           cos(q1)*(l4*cos(q2 + q3) - l3*sin(q2 + q3) + l2*cos(q2)),    //m(0,3)
           - cos(q1)*sin(q4) - cos(q2)*cos(q4)*sin(q1)*sin(q3) - cos(q3)*cos(q4)*sin(q1)*sin(q2),
           cos(q2)*sin(q1)*sin(q3)*sin(q4) - cos(q1)*cos(q4) + cos(q3)*sin(q1)*sin(q2)*sin(q4),
           cos(q2 + q3)*sin(q1),
           sin(q1)*(l4*cos(q2 + q3) - l3*sin(q2 + q3) + l2*cos(q2)),
           cos(q2 + q3)*cos(q4),
           -cos(q2 + q3)*sin(q4),
           sin(q2 + q3),
           l1 + l3*cos(q2 + q3) + l4*sin(q2 + q3) + l2*sin(q2),
           0,
           0,
           0,
           1;

    px = mht(0,3);
    py = mht (1,3);
    pz = mht(2,3);
}
