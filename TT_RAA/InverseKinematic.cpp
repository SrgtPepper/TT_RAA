#include "InverseKinematic.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <eigen3/Eigen/Dense>

using namespace Eigen;
using namespace std;

InverseKinematic::InverseKinematic()
{

}

InverseKinematic::~InverseKinematic()
{

}

double sinq3, cosq3, alfa, beta, px2, py2, pz2, l4, l9, c2q3;

void InverseKinematic::InverseKinect(double px, double py, double pz, double &q1, double &q2, double &q3){

    //Auxiliares para simplificar la ecuacion
    px2 = px*px;
    py2 = py*py;
    pz2 = (pz-l1)*(pz-l1);
    l4 = l2*l2;
    l9 = l3*l3;

    //Cos(q3)
    cosq3 = (px2 + py2 + pz2 - l4 - l9)/(2*l2*l3);

    if(cosq3 >= 1){
        cosq3 = 0.999999;
    }
    else if(cosq3 <= -1){
        cosq3 = -0.9999999;
    }

    c2q3 = cosq3*cosq3;

    //Sin(q3)
    sinq3 = ec*(sqrt(1-c2q3));

    //Auxiliares alfa y beta
    alfa = atan2(l3*sinq3 , l2+(l3*cosq3));
    beta = atan2(pz-l1, sqrt(px2 + py2));

    q1 = atan2(py , px);
    q2 = beta - alfa;
    q3 = atan2(sinq3 , cosq3);

    this -> q1 = q1;
    this -> q2 = q2;
    this -> q3 = q3;

}

void InverseKinematic::hitbox(double px, double py, double pz, bool &hit){

    float dist_origin_xy = sqrt(px*px+py*py);

    if ( dist_origin_xy <= r && pz <= h ){
        hit = true;
    }
    else{
        hit = false;
    }
}
