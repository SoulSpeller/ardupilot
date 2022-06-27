#pragma once

/// @file    AR_STA_Controller.h
/// @brief   sta Control algorithm. This is a instance of an
/// AR_STA_Controller class

/*
 * Originally written by hsp 2022
 */

#include <AP_Math/AP_Math.h>

class AR_STA_Controller {
public:
    AR_STA_Controller(float k3, float alpha, float lambda, float u1Sat, float voltSat) : 
    _coef_K3(k3), 
    _coef_alpha(alpha), 
    _coef_lambda(lambda), 
    _control_u1_Sat(u1Sat),
    _volt_Sat(voltSat),
    _control_u1(0.0),
    _control_u1_dot_pre(0.0) {

    }

    float update(float desired_yaw, float actual_yaw, float yaw_rate);

    void reset_Internal() {
        _control_u1 = 0.0;
        _control_u1_dot_pre = 0.0;
    }

    void set_dt(float dt) {
		_dt = dt;
	}

    float sign_val(float val) {
		return (val > FLT_EPSILON) ? 1.0 : -1.0;
	}

private:
    // coef
	float _coef_K3;
	float _coef_alpha;
    float _coef_lambda;
	float _control_u1_Sat;
    float _volt_Sat;;


	// internal variables
	float _dt;
    float _control_u1;
    float _control_u1_dot_pre;
};

