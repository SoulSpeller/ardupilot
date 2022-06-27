#include <AP_HAL/AP_HAL.h>
#include "AR_Los_Control.h"

extern const AP_HAL::HAL& hal;


float Slide_Ang_Estim::update(float x_error, float speed, float angle_ship_path)
{
	angle_ship_path = wrap_PI(angle_ship_path);
	
	float S1 = x_error;
	float S2_dot = speed * sinf(angle_ship_path) - _in_pre;
	_S2 = _S2 + (_S2_dot_pre + S2_dot) * 0.5f * _dt;
	_S2_dot_pre = S2_dot;
	float S = S1 + _S2;
	
	_in2 = _in2 + (_in2_dot_pre + sign_val(S)) * 0.5f * _dt;
	_in2_dot_pre = sign_val(S);
	float in = _coef_eta1 * safe_sqrt(fabsf(S)) * sign_val(S) + _coef_eta2 * _in2;
	_in_pre = in;
	
	float slide_angle = 0.0f;
	float divider = speed * cosf(angle_ship_path);
	if (is_equal(divider, 0.0f)) {
		return slide_angle;
	}
	
	slide_angle = in / divider;
	if (fabsf(slide_angle) > _angLimit) {
		slide_angle = _angLimit * sign_val(slide_angle);
	}
	return slide_angle;
}



/*
  Wrap AHRS yaw if in reverse - radians
 */
float AR_Los_Control::get_yaw() const
{
    if (_reverse) {
        return wrap_PI(M_PI + _ahrs.yaw);
    }
    return _ahrs.yaw;
}


/*
  Wrap AHRS yaw sensor if in reverse - centi-degress
 */
int32_t AR_Los_Control::get_yaw_sensor() const
{
    if (_reverse) {
        return wrap_180_cd(18000 + _ahrs.yaw_sensor);
    }
    return _ahrs.yaw_sensor;
}



/*
	not used in rover, return 0 directly
 */
int32_t AR_Los_Control::nav_roll_cd(void) const
{
    float ret = 0.0;
    return ret;
}



/*
  return the lateral acceleration needed to achieve tracking from the last
  update_*() operation
 */
float AR_Los_Control::lateral_acceleration(void) const
{
    return _latAccDem;
}

int32_t AR_Los_Control::nav_bearing_cd(void) const
{
    return wrap_180_cd(RadiansToCentiDegrees(_nav_bearing));
}

int32_t AR_Los_Control::bearing_error_cd(void) const
{
    return RadiansToCentiDegrees(_bearing_error);
}

int32_t AR_Los_Control::target_bearing_cd(void) const
{
    return wrap_180_cd(_target_bearing_cd);
}

/*
  this is the turn distance assuming a 90 degree turn
 */
float AR_Los_Control::turn_distance(float wp_radius) const
{
    wp_radius *= sq(_ahrs.get_EAS2TAS());
    return wp_radius;
}


/*
  this approximates the turn distance for a given turn angle. If the
  turn_angle is > 90 then a 90 degree turn distance is used, otherwise
  the turn distance is reduced linearly.
  This function allows straight ahead mission legs to avoid thinking
  they have reached the waypoint early, which makes things like camera
  trigger and ball drop at exact positions under mission control much easier
 */
float AR_Los_Control::turn_distance(float wp_radius, float turn_angle) const
{
    float distance_90 = turn_distance(wp_radius);
    turn_angle = fabsf(turn_angle);
    if (turn_angle >= 90) {
        return distance_90;
    }
    return distance_90 * turn_angle / 90.0f;
}

float AR_Los_Control::loiter_radius(const float radius) const
{
    float eas2tas_sq = sq(_ahrs.get_EAS2TAS());
        // Missing a sane input for calculating the limit, or the user has
        // requested a straight scaling with altitude. This will always vary
        // with the current altitude, but will at least protect the airframe
    return radius * eas2tas_sq;
}

bool AR_Los_Control::reached_loiter_target(void)
{
    return _WPcircle;
}


void AR_Los_Control::update_loiter(const struct Location &center_WP, float radius, int8_t loiter_direction)
{
	return;
}

void AR_Los_Control::update_heading_hold(int32_t navigation_heading_cd)
{
	return;
}

void AR_Los_Control::update_level_flight(void)
{
   return;
}

// update Los control for waypoint navigation
void AR_Los_Control::update_waypoint(const struct Location &prev_WP, const struct Location &next_WP, float dist_min)
{
    struct Location _current_loc;
	
	uint32_t now = AP_HAL::micros();
    float dt = (now - _last_update_waypoint_us) * 1.0e-6f;
	
    if (dt > 1) {
        // controller hasn't been called for an extended period of
        // time.  Reinitialise it.
        slideEstimator.reset_Internal();
    }
    if (dt > 0.1) {
        dt = 0.1;
    }
	
	_last_update_waypoint_us = now;
	
	// Get current position and velocity
    if (_ahrs.get_position(_current_loc) == false) {
        // if no GPS loc available, maintain last nav/target_bearing
        _data_is_stale = true;
        return;
    }
	
	Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();
	
	// update _target_bearing_cd
    _target_bearing_cd = _current_loc.get_bearing_to(next_WP);
	
	//Calculate groundspeed
    float groundSpeed = _groundspeed_vector.length();
    if (groundSpeed < 0.1f) {
        // use a small ground speed vector in the right direction,
        // allowing us to use the compass heading at zero GPS velocity
        groundSpeed = 0.1f;
        _groundspeed_vector = Vector2f(cosf(get_yaw()), sinf(get_yaw())) * groundSpeed;
    }
	
	// Calculate the NE position of WP B relative to WP A
    Vector2f AB = prev_WP.get_distance_NE(next_WP);
//    float AB_length = AB.length(); // unused variable
	
	// Check for AB zero length and track directly to the destination
    // if too small
    if (AB.length() < 1.0e-6f) {
        AB = _current_loc.get_distance_NE(next_WP);
        if (AB.length() < 1.0e-6f) {
            AB = Vector2f(cosf(get_yaw()), sinf(get_yaw()));
        }
    }
	// normolize AB so that distance calc by dot product and corss product need not divid norm of AB, norm(AB) is 1 already
    AB.normalize();
	
	
	// Calculate the NE position of the aircraft relative to WP A
    const Vector2f A_air = prev_WP.get_distance_NE(_current_loc);
	
	// calculate distance to target track, for reporting
	// % is the overloaded cross product
    _crosstrack_error = A_air % AB;
	
	
	//Determine if the aircraft is behind a +-135 degree degree arc centred on WP A
    //and further than L1 distance from WP A. Then use WP A as the L1 reference point
    //Otherwise do normal L1 guidance
//    float WP_A_dist = A_air.length(); // unused variable
    float alongTrackDist = A_air * AB;
	
	
	
	// key algorithm, step one, calc the los angle
	float Rlos = 0.0f;
	if (fabsf(_crosstrack_error) < _boat_length * _lookaway_coef) {
		Rlos = _boat_length * _lookaway_coef;
	}
	else {
		Rlos = _boat_length + fabsf(_crosstrack_error);
	}
	
	// AL means A(prevWp) to L(lospoint)
	float AL_length = fabsf(alongTrackDist) + safe_sqrt(Rlos * Rlos -  _crosstrack_error * _crosstrack_error);
	Vector2f AL = AB * AL_length;
	Vector2f Air_L = AL - A_air;
	_nav_bearing = Air_L.angle();
	
	
	// step two, estimate the slide angle
	slideEstimator.set_dt(dt);
	_slide_angle = slideEstimator.update(_crosstrack_error, groundSpeed, get_yaw()-AB.angle());
	
	// step three, compensate the los angle with slide angle
	_nav_bearing += _slide_angle;
	
	
	_bearing_error = _nav_bearing - get_yaw();
	_bearing_error = constrain_float(_bearing_error, -1.5708f, +1.5708f);
	_latAccDem = 0.0f;
	
	
    _WPcircle = false; // Waypoint capture status is always false during waypoint following
	_data_is_stale = false; // status are correctly updated with current waypoint data
}