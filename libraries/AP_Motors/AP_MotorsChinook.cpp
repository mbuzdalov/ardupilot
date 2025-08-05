/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_MotorsChinook.cpp - ArduCopter motors library for Chinook-style copters:
 *       two motors forward and rear, which can gimbal left and right,
 *       like a bicopter turned 90 degrees.
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsChinook.h"
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

#define SERVO_OUTPUT_RANGE  4500

// init
void AP_MotorsChinook::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // front throttle defaults to servo output 1
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor1, CH_1);

    // rear throttle defaults to servo output 2
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor2, CH_2);

    // front servo defaults to servo output 3
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor_tilt, CH_3);
    SRV_Channels::set_angle(SRV_Channel::k_motor_tilt, SERVO_OUTPUT_RANGE);

    // rear servo defaults to servo output 4
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorRear, CH_4);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorRear, SERVO_OUTPUT_RANGE);

    _mav_type = MAV_TYPE_VTOL_DUOROTOR;

    // record successful initialisation if what we setup was the desired frame_class
    set_initialised_ok(frame_class == MOTOR_FRAME_CHINOOK);
}


/// Constructor
AP_MotorsChinook::AP_MotorsChinook(uint16_t speed_hz) :
    AP_MotorsMulticopter(speed_hz)
{
    set_update_rate(speed_hz);
}


// set update rate to motors - a value in hertz
void AP_MotorsChinook::set_update_rate(uint16_t speed_hz)
{
    // record requested speed
    _speed_hz = speed_hz;

    SRV_Channels::set_rc_frequency(SRV_Channel::k_motor1, speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_motor2, speed_hz);
}

void AP_MotorsChinook::output_to_motors()
{
    if (!initialised_ok()) {
        return;
    }

    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
            _actuator[0] = 0.0f;
            _actuator[1] = 0.0f;
            SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt,    0.5 * (+_roll_radio_passthrough + _yaw_radio_passthrough) * SERVO_OUTPUT_RANGE);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear, 0.5 * (-_roll_radio_passthrough + _yaw_radio_passthrough) * SERVO_OUTPUT_RANGE);
            break;
        case SpoolState::GROUND_IDLE:
            set_actuator_with_slew(_actuator[0], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[1], actuator_spin_up_to_ground_idle());
            SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt,    _tilt_front * _spin_up_ratio * SERVO_OUTPUT_RANGE);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear, _tilt_rear * _spin_up_ratio * SERVO_OUTPUT_RANGE);
            break;
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            set_actuator_with_slew(_actuator[0], thr_lin.thrust_to_actuator(_thrust_front));
            set_actuator_with_slew(_actuator[1], thr_lin.thrust_to_actuator(_thrust_rear));
            SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt,    _tilt_front * SERVO_OUTPUT_RANGE);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear, _tilt_rear * SERVO_OUTPUT_RANGE);
            break;
    }

    SRV_Channels::set_output_pwm(SRV_Channel::k_motor1, output_to_pwm(_actuator[0]));
    SRV_Channels::set_output_pwm(SRV_Channel::k_motor2, output_to_pwm(_actuator[1]));
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint32_t AP_MotorsChinook::get_motor_mask()
{
    uint32_t motor_mask = 0;
    uint8_t chan;
    if (SRV_Channels::find_channel(SRV_Channel::k_motor1, chan)) {
        motor_mask |= 1U << chan;
    }
    if (SRV_Channels::find_channel(SRV_Channel::k_motor2, chan)) {
        motor_mask |= 1U << chan;
    }

    // add parent's mask
    motor_mask |= AP_MotorsMulticopter::get_motor_mask();

    return motor_mask;
}

void AP_MotorsChinook::assign_tilts(float unscaled_front, float unscaled_rear)
{
    _tilt_front = is_zero(_thrust_front) ? 0.0
            : constrain_float(unscaled_front / _thrust_front, -1.0, +1.0);
    _tilt_rear = is_zero(_thrust_rear) ? 0.0
            : constrain_float(unscaled_rear / _thrust_rear, -1.0, +1.0);
}

// calculate outputs to the motors
void AP_MotorsChinook::output_armed_stabilizing()
{
    // 0.1: Read out voltage and air pressure compensation.
    const float compensation_gain = thr_lin.get_compensation_gain();

    // 0.2: Initialize thrust requests.
    //      Throttle and pitch thrusts are scaled by compensation gain
    //        because motors are actually commanded in these units.
    //      Roll and yaw are also scaled,
    //        because servos control quantities like roll/throttle and yaw/throttle
    //        and we have to preserve this for the purpose of generating
    //        roll and yaw outputs that are as invariant in pressure and temperature
    //        as they can possibly be.
    float roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;
    float pitch_thrust = (_pitch_in + _pitch_in_ff) * compensation_gain;
    float yaw_thrust = (_yaw_in + _yaw_in_ff) * compensation_gain;
    float throttle_thrust = get_throttle() * compensation_gain;

    // 0.3: Hard max throttle: we can increase throttle_thrust, but never ever beyond this.
    const float throttle_hard_max = MIN(_throttle_avg_max, _throttle_thrust_max) * compensation_gain;

    // For this craft, we are working in small-angle approximations, which are somewhat OK
    //   if servo angles do not exceed 30 degrees in either direction.
    //   This means we assume sin(x) = x and cos(x) = 1.
    //   This simplifies equations and avoids trigonometry, which are both good for perf reasons,
    //   but obviously degrade control somewhat.

    // The equations are as follows:
    //  2*throttle = motor_front + motor_rear     // because throttle is average throttle
    //    pitch    = motor_front - motor_rear     // remember that positive pitch falls on the back
    //    roll     = motor_front * servo_front - motor_rear * servo_rear
    //    yaw      = motor_front * servo_front + motor_rear * servo_rear
    //
    // (servo directions are taken arbitrarily, but consistently with output_to_motors when SHUT_DOWN,
    //  so that one can test directions without arming)
    //
    // As a result, motors are uniquely determined by thrust and pitch,
    // roll and yaw then uniquely determine servo outputs.
    // But we still can decrease throttle arbitrarily, increase it up to max_throttle,
    // and we should adhere to the yaw headroom constraint, whatever it appears to be.
    //
    // It makes sense to divide roll, pitch and yaw by 2, so that equations get simpler.

    pitch_thrust *= 0.5;
    roll_thrust  *= 0.5;
    yaw_thrust   *= 0.5;

    // Now:
    //   2 * throttle = motor_front + motor_rear
    //   2 * pitch    = motor_front - motor_rear
    //   2 * roll     = motor_front * servo_front - motor_rear * servo_rear
    //   2 * yaw      = motor_front * servo_front + motor_rear * servo_rear
    //
    // and, conversely:
    //   motor_front = throttle + pitch                     // 0..1
    //   motor_rear  = throttle - pitch                     // 0..1
    //   servo_front = (yaw + roll) / (throttle + pitch)    // -1..+1
    //   servo_rear  = (yaw - roll) / (throttle - pitch)    // -1..+1

    // 1.1: If requested throttle is too high, have to cap it from above.
    if (throttle_thrust > throttle_hard_max) {
        throttle_thrust = throttle_hard_max;
        limit.throttle_upper = true;
    }

    // 1.2: If requested throttle is negative, have to cap it from below.
    if (throttle_thrust < 0) {
        throttle_thrust = 0;
        limit.throttle_lower = true;
    }

    // From constraints on motors we see that throttle cannot be smaller than |pitch|.
    // Also from constraints on the pitch and motors being at most 1,
    //   there are constraints from above on throttle: basically, throttle <= 1 - |pitch|.
    // From these and throttle being at most hard max, we can derive hard sanity limits on pitch.
    //
    // 1.3: Pitch greater than 0.5 cannot be delivered under any circumstances.
    //      Pitch greater than 2 * throttle hard max also cannot be delivered.
    const float max_pitch = MIN(0.5, 2 * throttle_hard_max);
    if (pitch_thrust > max_pitch) {
        pitch_thrust = max_pitch;
        limit.pitch = true;
    }
    if (pitch_thrust < -max_pitch) {
        pitch_thrust = -max_pitch;
        limit.pitch = true;
    }

    // 1.4: Both |yaw+roll| and |yaw-roll| cannot exceed throttle_hard_max each.
    //      As a result, either of the axes cannot exceed twice the hard max.
    const float max_yr = 2 * throttle_hard_max;
    if (roll_thrust > max_yr) {
        roll_thrust = max_yr;
        limit.roll = true;
    }
    if (roll_thrust < -max_yr) {
        roll_thrust = max_yr;
        limit.roll = true;
    }
    if (yaw_thrust > max_yr) {
        yaw_thrust = max_yr;
        limit.yaw = true;
    }
    if (yaw_thrust < -max_yr) {
        yaw_thrust = -max_yr;
        limit.yaw = true;
    }

    // From servo constraints we know that:
    //   -1 <= (yaw + roll) / (throttle + pitch) <= 1, which means:
    //      throttle + pitch >= |yaw + roll|, so
    //      throttle >= |yaw + roll| - pitch
    //   -1 <= (yaw - roll) / (throttle - pitch) <= 1, which means:
    //      throttle - pitch >= |yaw - roll|, so
    //      throttle >= |yaw - roll| + pitch
    // These new constraints subsume those coming from |pitch|.

    const float ypr = yaw_thrust + roll_thrust;
    const float ymr = yaw_thrust - roll_thrust;

    // 2.1: Compute soft minimum and soft maximum throttle.
    const float throttle_soft_min = MAX(fabsf(ypr) - pitch_thrust, fabsf(ymr) + pitch_thrust);
    const float throttle_soft_max = MIN(throttle_hard_max, 1.0 - fabsf(pitch_thrust));

    // 3: If throttle_soft_min <= throttle_soft_max, everything can be assigned directly.
    if (throttle_soft_min <= throttle_soft_max) {
        // 3.1: Throttle can be safely moved to the [soft min, soft max] region.
        if (throttle_thrust > throttle_soft_max) {
            throttle_thrust = throttle_soft_max;
            limit.throttle_upper = true;
        }
        if (throttle_thrust < throttle_soft_min) {
            throttle_thrust = throttle_soft_min;
            limit.throttle_lower = true;
        }

        // 3.2: Set outputs based on the equations.
        //       We must take care about zero motor outputs, because this is reachable
        //       from the disarmed state where all requests are zero.
        //
        // Here, applications of constrain_float protect from rounding errors only,
        // mathematically, they are not needed.
        _thrust_front = constrain_float(throttle_thrust + pitch_thrust, 0.0, 1.0);
        _thrust_rear = constrain_float(throttle_thrust - pitch_thrust, 0.0, 1.0);
        assign_tilts(ypr, ymr);

        // 3.3: This needs to be set so that filters work.
        //      This should be throttle_thrust / compensation_gain,
        //      but this is safer in the face of failed assertions.
        _throttle_out = (_thrust_front + _thrust_rear) / compensation_gain;
        return;
    }

    // 4: If we are here, we have to sacrifice something.
    //    If there is some nonzero yaw, we may first try to sacrifice some yaw,
    //    but not to the point where |yaw| is diminished below yaw headroom * |roll|.
    //    This reflects the idea behind yaw headroom that a certain fraction of actuators,
    //    servos in this case, should be reserved for some yaw authority.
    //
    // The entry condition is that |yaw| > 0, otherwise we don't have to care about it,
    //    and also |roll| > 0, otherwise we cannot shrink yaw meaningfully wrt yaw headroom.
    //
    // We may return from this "if", which means we found all assignments,
    //    or continue, which means that yaw and roll are now coupled,
    //    and everything should degrade even more gracefully.
    if (!is_zero(yaw_thrust) && !is_zero(roll_thrust)) {
        // 4.1: We try to leave pitch and roll intact.
        //      This means that we cannot exceed throttle_soft_max, as this will impact pitch,
        //      and other controls are strictly better with increased throttle at fixed pitch,
        //      so throttle would be throttle_soft_max.
        // This defines our motor outputs.
        _thrust_front = constrain_float(throttle_soft_max + pitch_thrust, 0.0, 1.0);
        _thrust_rear = constrain_float(throttle_soft_max - pitch_thrust, 0.0, 1.0);

        // 4.2: We interpret yaw headroom as how much |yaw|, in |roll| units, should remain.
        //      Also we are trying to find the minimum yaw value that is still good enough.
        const float yaw_headroom = constrain_float(_yaw_headroom * 0.001f, 0.0, 1.0);
        const float worst_abs_yaw = yaw_headroom * fabsf(roll_thrust);
        const float worst_yaw = yaw_thrust > 0 ? worst_abs_yaw : -worst_abs_yaw;

        // 4.3: This sets two constraints from below and two from above on what yaw can be
        //      for the servo outputs not to be exceeded.
        const float ymin = MAX(-_thrust_front - roll_thrust, -_thrust_rear + roll_thrust);
        const float ymax = MIN(+_thrust_front - roll_thrust, +_thrust_rear + roll_thrust);

        // 4.4: Try to find a good enough yaw.
        //      Note that yaw is always outside ymin...ymax, otherwise we should have not been here.
        //
        //  If ymin > ymax, then no good yaw exists.
        //  If ymin <= ymax, then the following cases exist:
        //      1a)   ymin <= ymax < worst_yaw <= yaw:    dead
        //      1b)   ymin <= worst_yaw <= ymax < yaw:    take ymax
        //      1c)   worst_yaw <= ymin <= ymax < yaw:    take ymax
        //      1d)   worst_yaw <= yaw < ymin <= ymax:    dead (but could consider taking ymin)
        //      2a)   ymin <= ymax < yaw <= worst_yaw:    dead (but could consider taking ymax)
        //      2b)   yaw < ymin <= ymax < worst_yaw:     take ymin
        //      2c)   yaw < ymin <= worst_yaw <= ymax:    take ymin
        //      2d)   yaw <= worst_yaw < ymin <= ymax:    dead
        float new_yaw = 0.0;
        if (ymin <= ymax) {
            if (yaw_thrust > ymax && ymax >= worst_yaw) { // 1b and 1c
                new_yaw = ymax;
            }
            if (yaw_thrust < ymin && ymin <= worst_yaw) { // 2b and 2c
                new_yaw = ymin;
            }
        }

        // 4.5: If a reduced yaw can be set, do it.
        //      We have already set the motors, just have to set servos and the total throttle.
        if (!is_zero(new_yaw)) {
            assign_tilts(new_yaw + roll_thrust, new_yaw - roll_thrust);
            _throttle_out = (_thrust_front + _thrust_rear) / compensation_gain;
            return;
        }

        // 4.6: Otherwise, we reduce the requested yaw to the worst allowed yaw.
        //      Note that it will be further scaled down in the subsequent code,
        //      however, the headroom will be preserved.
        yaw_thrust = worst_yaw;
    }

    // 5: If we reach this far, we failed to decrease |yaw| so that pitch and roll
    //    remain unchanged. Now we have to change pretty much everything, with an
    //    additional constraint that the yaw/roll ratio is now fixed.
    //
    // The equations are still:
    //   motor_front = throttle + pitch                     // 0..1
    //   motor_rear  = throttle - pitch                     // 0..1
    //   servo_front = (yaw + roll) / (throttle + pitch)    // -1..+1
    //   servo_rear  = (yaw - roll) / (throttle - pitch)    // -1..+1
    // with throttle now likely above soft max but below hard max.
    //
    // We introduce the relaxation coefficient Q in [0;1]
    // that will get applied to all axes except throttle.
    // We want to maximize it, which will minimize control discrepancies.
    //   motor_front = throttle + Q*pitch                     // 0..1
    //   motor_rear  = throttle - Q*pitch                     // 0..1
    //   servo_front = (yaw + roll) / (throttle/Q + pitch)    // -1..+1
    //   servo_rear  = (yaw - roll) / (throttle/Q - pitch)    // -1..+1
    //

    // 5.1: For 1/Q, to be minimized, the equations are as follows:
    //   1/Q >= |pitch| / throttle
    //   1/Q >= |pitch| / (1 - throttle)
    //   1/Q >= (|yaw + roll| - pitch) / throttle
    //   1/Q >= (|yaw - roll| + pitch) / throttle
    // The first is implied by the latter two.
    const float q_above_1mthrottle = fabsf(pitch_thrust);
    const float q_above_throttle = MAX(
        fabsf(yaw_thrust + roll_thrust) - pitch_thrust,
        fabsf(yaw_thrust - roll_thrust) + pitch_thrust
    );

    // 5.2: Our throttle should be not less than soft max, but not greater than hard max.
    //      Note: this does not divide by zero as the denominator is not zero
    //      once either pitch, roll, or yaw is nonzero.
    const float breakeven_throttle = q_above_throttle / (q_above_throttle + q_above_1mthrottle);
    throttle_thrust = constrain_float(breakeven_throttle, throttle_soft_max, throttle_hard_max);

    // 5.3: Determine the Q.
    //      q_above_throttle is nonzero, but q_above_1mthrottle can be zero.
    //      This only happens when pitch is zero, which means we don't have any components
    //      that care about throttle being too large.
    float q = throttle_thrust / q_above_throttle;
    if (!is_zero(q_above_1mthrottle)) {
        q = MIN(q, (1 - throttle_thrust) / q_above_1mthrottle);
    }

    // 5.4: Scale down the axes.
    pitch_thrust *= q;
    roll_thrust *= q;
    yaw_thrust *= q;

    // 5.5: Produce the outputs.
    _thrust_front = constrain_float(throttle_thrust + pitch_thrust, 0.0, 1.0);
    _thrust_rear = constrain_float(throttle_thrust - pitch_thrust, 0.0, 1.0);
    assign_tilts(yaw_thrust + roll_thrust, yaw_thrust - roll_thrust);
    _throttle_out = (_thrust_front + _thrust_rear) / compensation_gain;
}

// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsChinook::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // output to motors and servos
    switch (motor_seq) {
        case 1:
            // forward throttle
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor1, pwm);
            break;
        case 2:
            // rear throttle
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor2, pwm);
            break;
        case 3:
            // front servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor_tilt, pwm);
            break;
        case 4:
            // rear servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltMotorRear, pwm);
            break;
        default:
            // do nothing
            break;
    }
}
