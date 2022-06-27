#pragma once

/// @file    AR_Los_Control.h
/// @brief   Los Control algorithm. This is a instance of an
/// AP_Navigation class

/*
 * Originally written by hsp 2022
 */

#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <AP_Navigation/AP_Navigation.h>
//#include <AP_SpdHgtControl/AP_SpdHgtControl.h>
#include <AP_Common/Location.h>


class Slide_Ang_Estim {
public:
	Slide_Ang_Estim(float uncertain, float angLimit) {
		_uncertain = uncertain;
		_coef_eta1 = 1.5 * safe_sqrt(_uncertain);
		_coef_eta2 = 4 * _uncertain;
		_angLimit = angLimit;
		
		_S2 = 0;
		_S2_dot_pre = 0;
		_in2 = 0;
		_in2_dot_pre = 0;
		_in_pre = 0;
	}
	
	void set_dt(float dt) {
		_dt = dt;
	}
	
	void reset_Internal() {
		_S2 = 0;
		_S2_dot_pre = 0;
		_in2 = 0;
		_in2_dot_pre = 0;
		_in_pre = 0;
	}
	
	float update(float x_error, float speed, float angle_ship_path);
	
	float sign_val(float val) {
		return (val > FLT_EPSILON) ? 1.0 : -1.0;
	}
	
private:
	// coef
	float _uncertain;
	float _coef_eta1;
	float _coef_eta2;
	float _angLimit;

	// internal variables
	float _dt;
	float _S2;
	float _S2_dot_pre;
	float _in2;
	float _in2_dot_pre;
	float _in_pre;	
};




class AR_Los_Control : public AP_Navigation {
public:
	AR_Los_Control(AP_AHRS &ahrs):_ahrs(ahrs) {
		_boat_length = 1.0;
		_lookaway_coef = 2.0;
	}
	
	/* Do not allow copies */
    AR_Los_Control(const AR_Los_Control &other) = delete;
    AR_Los_Control &operator=(const AR_Los_Control&) = delete;
	
	/* see AP_Navigation.h for the definitions and units of these
     * functions */
    int32_t nav_roll_cd(void) const override;
    float lateral_acceleration(void) const override;
	
	// return the desired track heading angle(centi-degrees)
    int32_t nav_bearing_cd(void) const override;
	
	// return the heading error angle (centi-degrees) +ve to left of track
    int32_t bearing_error_cd(void) const override;
	
	float crosstrack_error(void) const override { return _crosstrack_error; }
    float crosstrack_error_integrator(void) const override { return _crosstrack_error; }
	
	int32_t target_bearing_cd(void) const override;
    float turn_distance(float wp_radius) const override;
    float turn_distance(float wp_radius, float turn_angle) const override;
    float loiter_radius (const float loiter_radius) const override;
    void update_waypoint(const struct Location &prev_WP, const struct Location &next_WP, float dist_min = 0.0f) override;
    void update_loiter(const struct Location &center_WP, float radius, int8_t loiter_direction) override;
    void update_heading_hold(int32_t navigation_heading_cd) override;
    void update_level_flight(void) override;
    bool reached_loiter_target(void) override;
	
	
	// notify Navigation controller that a new waypoint has just been
    // processed. This means that until we handle an update_XXX() function
    // the data is stale with old navigation information.
	void set_data_is_stale(void) override {
        _data_is_stale = true;
    }
	// return true if a new waypoint has been processed by mission
    // controller but the navigation controller still has old stale data
    // from previous waypoint navigation handling. This gets cleared on
    // every update_XXXXXX() call.
    bool data_is_stale(void) const override {
        return _data_is_stale;
    }
	
	void set_reverse(bool reverse) override {
        _reverse = reverse;
    }

private:
	Slide_Ang_Estim slideEstimator{0.5, 0.36};
	
	// internal parameters
	float _boat_length;
	float _lookaway_coef;
	
	// slide angle is positive+ when actual speed in the left of ship yaw
	float _slide_angle;
	
	// reference to the AHRS object
    AP_AHRS &_ahrs;
	
	// lateral acceration in m/s required to go to the los point
    float _latAccDem;
	
	// Status which is true when the vehicle has started circling the WP
    bool _WPcircle;
	
	// bearing angle (radians) to LOS point
    float _nav_bearing;
	
	// bearing error angle (radians) +ve to left of track
    float _bearing_error;
	
	// crosstrack error in meters
    float _crosstrack_error;
	
	// target bearing in centi-degrees from last update
    int32_t _target_bearing_cd;

	bool _data_is_stale = true;	

    uint32_t _last_update_waypoint_us;
	
	bool _reverse = false;
    float get_yaw() const;
    int32_t get_yaw_sensor() const;
};