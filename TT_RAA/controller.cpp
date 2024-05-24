#include "controller.h"

Controller::Controller() {

}
Controller::~Controller() {

}



void Controller::PVG(Vector4d desired_pos, Vector4d q, Vector4d qp, Vector4d Phi, Vector4d &Tau){
    this->pos_error = desired_pos-q;

    Tau = Kp*pos_error - Kd*qp + Phi ;
}
void Controller::set_kp_gains(Matrix4d kp_new)
{
    this->Kp = kp_new;
}
void Controller::set_kd_gains(Matrix4d kd_new)
{
    this->Kd = kd_new;
}
