#include "InverseKinematic.h"
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

double sinq3, cosq3, alfa, beta;


void InverseKinematic::InverseKinect(double px, double py, double pz, double &q1, double &q2, double &q3){

    //Estos condicionales son para evitar que los valores de px, py y pz sean 0
    if(px == 0){
        px = 0.000001;
    }

    //Estos valores de seno, coseno, alfa y beta sirven como auxiliares
    cosq3 = (((px*px) + (py*py) + ((pz-l1)*(pz-l1)) - (l2*l2) - ((l3+l4)*(l3+l4)))/(2*l2*(l3+l4)));

    if(cosq3 == 1){
        cosq3 = 0.99999999999;
    }

    sinq3 = sqrt(1-(cosq3*cosq3));

    if(sinq3 == 1){
        sinq3 = 0.99999999999;
    }

    alfa = atan2(((l3+l4)*sinq3),(l2 + ((l3+l4)*cosq3)));

    beta = atan2((pz-l1),sqrt((px*px)+(py*py)));


    //Aquí los valores de los angulos
    q1 = atan2(py,px);

    q3 = atan2(sinq3,cosq3);

    q2 = beta - alfa;

    this -> q1 = q1;
    this -> q2 = q2;
    this -> q3 = q3;

}
