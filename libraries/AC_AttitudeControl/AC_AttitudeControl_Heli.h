// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file    AC_AttitudeControl_Heli.h
/// @brief   ArduCopter attitude control library for traditional helicopters

#ifndef AC_ATTITUDECONTROL_HELI_H
#define AC_ATTITUDECONTROL_HELI_H

#include "AC_AttitudeControl.h"
#include <AP_Motors/AP_MotorsHeli.h>
#include <AC_PID/AC_HELI_PID.h>
#include <Filter/Filter.h>

#define AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE  0.02f
#define AC_ATTITUDE_HELI_RATE_RP_FF_FILTER          10.0f
#define AC_ATTITUDE_HELI_RATE_Y_VFF_FILTER          10.0f

# define AC_ATTITUDE_HELI_STAB_COLLECTIVE_MIN_DEFAULT     0
# define AC_ATTITUDE_HELI_STAB_COLLECTIVE_LOW_DEFAULT     400
# define AC_ATTITUDE_HELI_STAB_COLLECTIVE_HIGH_DEFAULT    600
# define AC_ATTITUDE_HELI_STAB_COLLECTIVE_MAX_DEFAULT     1000

class AC_AttitudeControl_Heli : public AC_AttitudeControl {
public:
    AC_AttitudeControl_Heli( AP_AHRS &ahrs,
                        const AP_Vehicle::MultiCopter &aparm,
                        AP_MotorsHeli& motors,
                        AC_P& p_angle_roll, AC_P& p_angle_pitch, AC_P& p_angle_yaw,
                        AC_HELI_PID& pid_rate_roll, AC_HELI_PID& pid_rate_pitch, AC_HELI_PID& pid_rate_yaw
                        ) :
        AC_AttitudeControl(ahrs, aparm, motors,
                           p_angle_roll, p_angle_pitch, p_angle_yaw,
                           pid_rate_roll, pid_rate_pitch, pid_rate_yaw),
        _passthrough_roll(0), _passthrough_pitch(0), _passthrough_yaw(0),
        pitch_feedforward_filter(AC_ATTITUDE_HELI_RATE_RP_FF_FILTER),
        roll_feedforward_filter(AC_ATTITUDE_HELI_RATE_RP_FF_FILTER),
        yaw_velocity_feedforward_filter(AC_ATTITUDE_HELI_RATE_Y_VFF_FILTER)
        {
            AP_Param::setup_object_defaults(this, var_info);

            // initialise flags
            _flags_heli.limit_roll = false;
            _flags_heli.limit_pitch = false;
            _flags_heli.limit_yaw = false;
            _flags_heli.leaky_i = true;
            _flags_heli.flybar_passthrough = false;
            _flags_heli.tail_passthrough = false;
            _flags_heli.do_piro_comp = false;
        }

    // passthrough_bf_roll_pitch_rate_yaw - roll and pitch are passed through directly, body-frame rate target for yaw
    void passthrough_bf_roll_pitch_rate_yaw(float roll_passthrough, float pitch_passthrough, float yaw_rate_bf);

	// rate_controller_run - run lowest level body-frame rate controller and send outputs to the motors
	// should be called at 100hz or more
	virtual void rate_controller_run();

	// use_leaky_i - controls whether we use leaky i term for body-frame to motor output stage
	void use_leaky_i(bool leaky_i) {  _flags_heli.leaky_i = leaky_i; }
    
    // use_flybar_passthrough - controls whether we pass-through
    // control inputs to swash-plate and tail
    void use_flybar_passthrough(bool passthrough, bool tail_passthrough) {  
        _flags_heli.flybar_passthrough = passthrough; 
        _flags_heli.tail_passthrough = tail_passthrough; 
    }

    // do_piro_comp - controls whether piro-comp is active or not
    void do_piro_comp(bool piro_comp) { _flags_heli.do_piro_comp = piro_comp; }

    // get_pilot_desired_collective - rescale's pilot collective pitch input in Stabilize and Acro modes
    int16_t get_pilot_desired_collective(int16_t control_in);

    // set_use_stab_col - setter function
    void set_use_stab_col(bool use) { _flags_heli.use_stab_col = use; }

    // set_heli_stab_col_ramp - setter function
    void set_stab_col_ramp(float ramp) { _stab_col_ramp = constrain_float(ramp, 0.0, 1.0); }

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

private:

    // To-Do: move these limits flags into the heli motors class
    struct AttControlHeliFlags {
        uint8_t limit_roll          :   1;  // 1 if we have requested larger roll angle than swash can physically move
        uint8_t limit_pitch         :   1;  // 1 if we have requested larger pitch angle than swash can physically move
        uint8_t limit_yaw           :   1;  // 1 if we have requested larger yaw angle than tail servo can physically move
        uint8_t leaky_i             :   1;  // 1 if we should use leaky i term for body-frame rate to motor stage
        uint8_t flybar_passthrough  :   1;  // 1 if we should pass through pilots roll & pitch input directly to swash-plate
        uint8_t tail_passthrough    :   1;  // 1 if we should pass through pilots yaw input to tail
        uint8_t do_piro_comp        :   1;  // 1 if we should do pirouette compensation on roll/pitch
        uint8_t use_stab_col        :   1;  // 1 if we should use Stabilise mode collective range, 0 for Acro range
    } _flags_heli;

    //
    // body-frame rate controller
    //
	// rate_bf_to_motor_roll_pitch - ask the rate controller to calculate the motor outputs to achieve the target body-frame rate (in centi-degrees/sec) for roll, pitch and yaw
    // outputs are sent directly to motor class
    void rate_bf_to_motor_roll_pitch(float rate_roll_target_cds, float rate_pitch_target_cds);
    virtual float rate_bf_to_motor_yaw(float rate_yaw_cds);

    //
    // throttle methods
    //

    // calculate total body frame throttle required to produce the given earth frame throttle
    float get_boosted_throttle(float throttle_in);
    
    // pass through for roll and pitch
    int16_t _passthrough_roll;
    int16_t _passthrough_pitch;

    // pass through for yaw if tail_passthrough is set
    int16_t _passthrough_yaw;

    //  factor used to smoothly ramp collective from Acro value to Stab-Col value
    float _stab_col_ramp;

    // parameters
    AP_Int8         _piro_comp_enabled;             // Flybar present or not.  Affects attitude controller used during ACRO flight mode
    AP_Int16        _heli_stab_col_min;             // minimum collective pitch setting at zero throttle input in Stabilize mode
    AP_Int16        _heli_stab_col_low;             // collective pitch setting at mid-low throttle input in Stabilize mode
    AP_Int16        _heli_stab_col_high;            // collective pitch setting at mid-high throttle input in Stabilize mode
    AP_Int16        _heli_stab_col_max;             // maximum collective pitch setting at full throttle input in Stabilize mode
    AP_Float        _acro_col_expo;                 // used to soften collective pitch inputs near center point in Acro mode.
    
    // LPF filters to act on Rate Feedforward terms to linearize output.
    // Due to complicated aerodynamic effects, feedforwards acting too fast can lead
    // to jerks on rate change requests.
    LowPassFilterFloat pitch_feedforward_filter;
    LowPassFilterFloat roll_feedforward_filter;
    LowPassFilterFloat yaw_velocity_feedforward_filter;

};

#endif //AC_ATTITUDECONTROL_HELI_H
