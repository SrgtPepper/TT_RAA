#include "Kinematic.h"
#include <eigen3/Eigen/Dense>
#include <math.h>
#define PI 3.141594

using namespace Eigen;
using namespace std;

Matrix4f mht;   //Declaracion de la matriz homogenea total

Kinematic::Kinematic()
{

}


Kinematic::~Kinematic()
{

}



void Kinematic::DirecKinect(double q1,double q2,double q3, double &px, double &py, double &pz){

    px = cos(q1)*(l2*cos(q2) + (l3+l4)*cos(q2+q3));
    py = sin(q1)*(l2*cos(q2) + (l3+l4)*cos(q2+q3));
    pz = l1 + l2*sin(q2) + (l3+l4)*sin(q2+q3);

    this -> px = px;
    this -> py = py;
    this -> pz = pz;
}
