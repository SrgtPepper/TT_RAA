#include "integrator.h"

Integrator::Integrator()
{

}
Integrator::~Integrator()
{

}

void Integrator::Integral_Euler(Vector4d in, Vector4d &out)
{
    for(int i=0;i<4;i++){
        out[i] = this->aux[i] + h*in[i];
    }
    this->aux = out;
}

void Integrator::setIntegrationStep(double h)
{
    this->h = h;
}
