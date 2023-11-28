#include "InverseKinematic.h"
#include <eigen3/Eigen/Dense>

using namespace Eigen;
using namespace std;

InverseKinematic::InverseKinematic()
{

}

InverseKinematic::~InverseKinematic()
{

}

double sinq3, cosq3;


void InverseKinematic::InverseKinect(double px, double py, double pz, double &q1, double &q2, double &q3){

    cosq3 = ((px*px)+(py*py)+(pz*pz)-(l2*l2)-(l3*l3))/(2*l2*l3);

    sinq3 = sqrt(1-(cosq3*cosq3));

    q1 = atan(py/px);

    q2 = atan(pz/sqrt((px*px)+(py*py))) - atan((l3*sinq3/(l2+(l3*cosq3))));

    q3 = atan((sqrt(1-(cosq3*cosq3))/(cosq3)));

}
