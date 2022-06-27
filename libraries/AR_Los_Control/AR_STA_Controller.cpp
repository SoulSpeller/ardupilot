#include "AR_STA_Controller.h"


float AR_STA_Controller::update(float desired_yaw, float actual_yaw, float yaw_rate) {
    float phi_error = wrap_PI(actual_yaw - desired_yaw);
    float s_mani = _coef_K3 * phi_error + yaw_rate;
    float u1_dot = -1.0 * _coef_alpha * sign_val(s_mani);

    _control_u1 = _control_u1 + ((u1_dot + _control_u1_dot_pre) * 0.5 * _dt);
    _control_u1_dot_pre = u1_dot;

    if (fabsf(_control_u1) > _control_u1_Sat) {
        _control_u1 = sign_val(_control_u1) * _control_u1_Sat;
    }

    float controlOutPut = -1.0 * _coef_lambda * safe_sqrt(fabsf(s_mani)) * sign_val(s_mani) + _control_u1;
    if (fabsf(controlOutPut) > _volt_Sat) {
        controlOutPut = sign_val(controlOutPut) * _volt_Sat;
    }

    return (controlOutPut / _volt_Sat);
}