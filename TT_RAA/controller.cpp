#include "controller.h"

Controller::Controller() {

}
Controller::~Controller() {

}


// Metodo del controlador proporcional con realimentacion de velocidad y compensacion de gravedad
void Controller::PVG(Vector4d error, Vector4d qp, Vector4d Phi, Vector4d &Tau){

    Tau = Kp*error - Kd*qp + Phi;

}
// Metodo para establecer la matriz de ganancias de posicion
void Controller::set_kp_gains(Matrix4d kp_new)
{
    this->Kp = kp_new;
}
// Metodo para establecer la matriz de ganancias de velocidad
void Controller::set_kd_gains(Matrix4d kd_new)
{
    this->Kd = kd_new;
}
