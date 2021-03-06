/// @file	AC_PID_2D.cpp
/// @brief	Generic PID algorithm

#include <AP_Math/AP_Math.h>
#include "AC_PID_2D.h"

#define AC_PID_2D_FILT_HZ_DEFAULT  20.0f   // default input filter frequency
#define AC_PID_2D_FILT_HZ_MIN      0.01f   // minimum input filter frequency
#define AC_PID_2D_FILT_D_HZ_DEFAULT  10.0f   // default input filter frequency
#define AC_PID_2D_FILT_D_HZ_MIN      0.005f   // minimum input filter frequency


// Constructor
AC_PID_2D::AC_PID_2D(float initial_p, float initial_i, float initial_d, float initial_imax, float initial_filt_hz, float initial_filt_d_hz, float dt) :
    _dt(dt)
{
    // load parameter values from eeprom
//    AP_Param::setup_object_defaults(this, var_info);

    _kp = initial_p;
    _ki = initial_i;
    _kd = initial_d;
    _imax = fabsf(initial_imax);
    filt_hz(initial_filt_hz);
    filt_d_hz(initial_filt_d_hz);

    // reset input filter to first value received
    _flags._reset_filter = true;
}

// set_dt - set time step in seconds
void AC_PID_2D::set_dt(float dt)
{
    // set dt and calculate the input filter alpha
    _dt = dt;
    calc_filt_alpha();
    calc_filt_alpha_d();
}

// filt_hz - set input filter hz
void AC_PID_2D::filt_hz(float hz)
{
    _filt_hz = fabsf(hz);

    // sanity check _filt_hz
    _filt_hz = MAX(_filt_hz, AC_PID_2D_FILT_HZ_MIN);

    // calculate the input filter alpha
    calc_filt_alpha();
}

// filt_d_hz - set input filter hz
void AC_PID_2D::filt_d_hz(float hz)
{
    _filt_d_hz = fabsf(hz);

    // sanity check _filt_hz
    _filt_d_hz = MAX(_filt_d_hz, AC_PID_2D_FILT_D_HZ_MIN);

    // calculate the input filter alpha
    calc_filt_alpha_d();
}

// set_input - set input to PID controller
//  input is filtered before the PID controllers are run
//  this should be called before any other calls to get_p, get_i or get_d
void AC_PID_2D::set_input(const Vector2f &input)
{
    // don't process inf or NaN
    if (!isfinite(input.x) || !isfinite(input.y)) {
        return;
    }

    // reset input filter to value received
    if (_flags._reset_filter) {
        _flags._reset_filter = false;
        _input = input;
    }

    // update filter and calculate derivative
    const Vector2f input_delta = (input - _input) * _filt_alpha;
    _input += input_delta;

    set_input_filter_d(input_delta);
}

// set_input_filter_d - set input to PID controller
//  only input to the D portion of the controller is filtered
//  this should be called before any other calls to get_p, get_i or get_d
void AC_PID_2D::set_input_filter_d(const Vector2f& input_delta)
{
    // don't process inf or NaN
    if (!isfinite(input_delta.x) && !isfinite(input_delta.y)) {
        return;
    }

    // reset input filter
    if (_flags._reset_filter) {
        _flags._reset_filter = false;
        _derivative.x = 0.0f;
        _derivative.y = 0.0f;
    }

    // update filter and calculate derivative
    if (is_positive(_dt)) {
        const Vector2f derivative = input_delta / _dt;
        const Vector2f delta_derivative = (derivative - _derivative) * _filt_alpha_d;
        _derivative += delta_derivative;
    }
}

Vector2f AC_PID_2D::get_p() const
{
    return (_input * _kp);
}

Vector2f AC_PID_2D::get_i()
{
    if (!is_zero(_ki) && !is_zero(_dt)) {
        _integrator += (_input * _ki) * _dt;
        const float integrator_length = _integrator.length();
        if ((integrator_length > _imax) && is_positive(integrator_length)) {
            _integrator *= (_imax / integrator_length);
        }
        return _integrator;
    }
    return Vector2f();
}

// get_i_shrink - get_i but do not allow integrator to grow in length (it may shrink)
Vector2f AC_PID_2D::get_i_shrink()
{
    if (!is_zero(_ki) && !is_zero(_dt)) {
        const float integrator_length_orig = MIN(_integrator.length(), _imax);
        _integrator += (_input * _ki) * _dt;
        const float integrator_length_new = _integrator.length();
        if ((integrator_length_new > integrator_length_orig) && is_positive(integrator_length_new)) {
            _integrator *= (integrator_length_orig / integrator_length_new);
        }
        return _integrator;
    }
    return Vector2f();
}

Vector2f AC_PID_2D::get_d()
{
    // derivative component
    return Vector2f(_kd * _derivative.x, _kd * _derivative.y);
}

Vector2f AC_PID_2D::get_pid()
{
    return get_p() + get_i() + get_d();
}

void AC_PID_2D::reset_I()
{
    _integrator.zero();
}
void AC_PID_2D::reset_filter()
{
	_flags._reset_filter = true;
    _derivative.zero();
    _integrator.zero();
}

void AC_PID_2D::load_gains()
{
//    _kp.load();
//    _ki.load();
//    _kd.load();
//    _imax.load();
//    _imax = fabsf(_imax);
//    _filt_hz.load();
//    _filt_d_hz.load();

//    // calculate the input filter alpha
//    calc_filt_alpha();
//    calc_filt_alpha_d();
}

// save_gains - save gains to eeprom
void AC_PID_2D::save_gains()
{
//    _kp.save();
//    _ki.save();
//    _kd.save();
//    _imax.save();
//    _filt_hz.save();
//    _filt_d_hz.save();
}

// calc_filt_alpha - recalculate the input filter alpha
void AC_PID_2D::calc_filt_alpha()
{
    if (is_zero(_filt_hz)) {
        _filt_alpha = 1.0f;
        return;
    }
  
    // calculate alpha
    const float rc = 1/(M_2PI*_filt_hz);
    _filt_alpha = _dt / (_dt + rc);
}

// calc_filt_alpha - recalculate the input filter alpha
void AC_PID_2D::calc_filt_alpha_d()
{
    if (is_zero(_filt_d_hz)) {
        _filt_alpha_d = 1.0f;
        return;
    }

    // calculate alpha
    const float rc = 1/(M_2PI*_filt_d_hz);
    _filt_alpha_d = _dt / (_dt + rc);
}
