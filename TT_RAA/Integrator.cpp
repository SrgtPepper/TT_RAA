#include <Integrator.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>


using namespace Eigen;
using namespace std;

Integrator::Integrator(){

}

Integrator::~Integrator(){

}

Vector4d Integrator::integral(Vector4d values,Vector4d &aux){

    Vector4d result = aux+h*values;
    aux = result;
    return result;

}
