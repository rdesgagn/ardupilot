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

#include "AP_MotorsMatrix_Optimal.h"

#pragma GCC diagnostic error "-Wframe-larger-than=2500"

void AP_MotorsMatrix_Optimal::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    AP_MotorsMatrix::init(frame_class, frame_type);
    if (!initialised_ok()) {
        // underlying class must init correctly
        return;
    }

    if (frame_class == MOTOR_FRAME_SCRIPTING_MATRIX) {
        // Scripting frame class not supported
        // Scripting is setup to use the AP_MotorsMatrix singleton, which wont exist if were here.
        // So there is no way for scripting to setup the motors, once we fix that it should work well...
        set_initialised_ok(false);
        return;
    }

    // conversion factors so the new mix comes out close to the old
    // should reduce the need to re-tune, I have not calculated them all yet...
    // there may be some way to derive from the motor matrix, but I can't work it out
    // these are still not a perfect conversion, I'm not sure why....
    // may also need a yaw conversion for the A V and tail frames... but there quads so this mixer won't help much in anycase
    float roll_conversion; 
    float pitch_conversion;
    switch (frame_class) {
        case MOTOR_FRAME_QUAD: {
            switch (frame_type) {
                case MOTOR_FRAME_TYPE_PLUS:
                    roll_conversion = 0.5;
                    pitch_conversion = 0.5;
                    break;
                case MOTOR_FRAME_TYPE_X:
                    roll_conversion = 1.0;
                    pitch_conversion = 1.0;
                    break;
                default:
                    return;
            }
            break;
        }
        case MOTOR_FRAME_HEXA: {
            switch (frame_type) {
                 case MOTOR_FRAME_TYPE_X:
                    roll_conversion = 0.75;
                    pitch_conversion = 1.0;
                    break;
                default:
                    return;
            }
            break;
        }
        case MOTOR_FRAME_OCTA: {
            switch (frame_type) {
                case MOTOR_FRAME_TYPE_X:
                    roll_conversion = 1.1715728;
                    pitch_conversion = 1.1715728;
                    break;
                default:
                    return;
            }
            break;
        }
        case MOTOR_FRAME_OCTAQUAD: {
            switch (frame_type) {
                case MOTOR_FRAME_TYPE_X:
                    roll_conversion = 2.0;
                    pitch_conversion = 2.0;
                    break;
                default:
                    return;
            }
            break;
        }
        case MOTOR_FRAME_DODECAHEXA: {
            switch (frame_type) {
                case MOTOR_FRAME_TYPE_X:
                    roll_conversion = 1.5;
                    pitch_conversion = 2.0;
                    break;
                default:
                    return;
            }
            break;
        }
        default:
            return;
    }

    // convert motor factors to matrix format
    uint8_t num_motors = 0;
    for (uint8_t i = 0; i < max_num_motors; i++) {
        if (motor_enabled[i]) {
            motor_factors.v[i][0] = _roll_factor[i] / roll_conversion;
            motor_factors.v[i][1] = _pitch_factor[i] / pitch_conversion;
            motor_factors.v[i][2] = _yaw_factor[i];
            motor_factors.v[i][3] = _throttle_factor[i];
            num_motors++;
        }
    }

    // Weighting vector defines the relative weighting of roll, pitch, yaw and throttle
    // may want to change this on the fly in the future, but in that case we can no longer pre-compute the hessian
    MatrixRC<float,1,4> w;
    w.v[0][0] = 15.0; // roll
    w.v[0][1] = 15.0; // pitch
    w.v[0][2] = 15.0; // yaw
    w.v[0][3] = 1.0;  // throttle

    // setup hessian matrix
    H = matrix_multiply(motor_factors.per_element_mult_vector_columns(w), motor_factors.transposed());
    for (uint8_t i = num_motors; i < max_num_motors; i++) {
        H.v[i][i] = 1.0; // populate remaining diagonals to ensure positive definite matrix
    }

    // setup constraints
    for (uint8_t i = 0; i < max_num_motors; i++) {
        A.average_throttle.v[i][0] = -motor_factors.v[i][3] / num_motors;
        if (motor_enabled[i]) {
            A.max_throttle.v[i][0] = -1.0;
            A.min_throttle.v[i][0] =  1.0;
        }
    }

    // re-scale by weights to save runtime calculation
    w.v[0][3] *= num_motors;
    w *= -1;
    motor_factors = motor_factors.per_element_mult_vector_columns(w);

}


// output - sends commands to the motors, 
void AP_MotorsMatrix_Optimal::output_armed_stabilizing()
{
    if (!initialised_ok()) {
        AP_MotorsMatrix::output_armed_stabilizing();
        return;
    }

    // apply voltage and air pressure compensation
    const float compensation_gain = get_compensation_gain(); // compensation for battery voltage and altitude
    const float roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;
    const float pitch_thrust = (_pitch_in + _pitch_in_ff) * compensation_gain;
    const float yaw_thrust = (_yaw_in + _yaw_in_ff) * compensation_gain;
    float throttle_thrust = get_throttle() * compensation_gain;
    float throttle_avg_max = _throttle_avg_max * compensation_gain;

    // If thrust boost is active then do not limit maximum thrust
    const float throttle_thrust_max = _thrust_boost_ratio + (1.0 - _thrust_boost_ratio) * _throttle_thrust_max * compensation_gain;

    // sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= 0.0) {
        throttle_thrust = 0.0;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= throttle_thrust_max) {
        throttle_thrust = throttle_thrust_max;
        limit.throttle_upper = true;
    }

    // ensure that throttle_avg_max is between the input throttle and the maximum throttle
    throttle_avg_max = constrain_float(throttle_avg_max, throttle_thrust, throttle_thrust_max);

    // set roll, pitch, yaw and throttle input value
    MatrixRC<float,4,1> input;
    input.v[0][0] = roll_thrust;
    input.v[1][0] = pitch_thrust;
    input.v[2][0] = yaw_thrust;
    input.v[3][0] = throttle_thrust;

    // input matrix
    MatrixRC<float,max_num_motors,1> f = matrix_multiply(motor_factors, input);

    // constraints, average throttle, 1 and 0
    MatrixRC<float,num_constraints,1> b;
    b.v[0][0] = -throttle_avg_max;
    for (uint8_t i = 0; i < max_num_motors; i++) {
        if (motor_enabled[i]) {
            b.v[1+i][0] = -1.0; // could set this to 0 if a failed motor is detected
        }
    }

    // the clever bit
    interior_point_solve(f, b);

    // copy to motor outputs
    for (uint8_t i = 0; i < max_num_motors; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = x.v[i][0];
        }
    }

    // note that this does not do anything with the limit flags, we could set a threshold on desired vs achieved output.
    // hopefully limit flags will be triggered much less often in anycase...
}

// sparse A matrix handling
// x = A * B
MatrixRC<float,AP_MotorsMatrix_Optimal::max_num_motors,1> AP_MotorsMatrix_Optimal::A_mult(const MatrixRC<float,num_constraints,1>& B) const
{
    MatrixRC<float,max_num_motors,1> X;
    for (uint8_t i = 0; i < max_num_motors; i++) {
        X.v[i][0] = A.average_throttle.v[i][0]*B.v[0][0] + A.max_throttle.v[i][0]*B.v[1+i][0] + A.min_throttle.v[i][0]*B.v[1+max_num_motors+i][0];
    }
    return X;
}

// x = A' * B
MatrixRC<float,AP_MotorsMatrix_Optimal::num_constraints,1> AP_MotorsMatrix_Optimal::At_mult(const MatrixRC<float,max_num_motors,1>& B) const
{
    MatrixRC<float,num_constraints,1> X;
    X.v[0][0] = A.average_throttle.dot(B);
    for (uint8_t i = 0; i < max_num_motors; i++) {
        X.v[1+i][0] = A.max_throttle.v[i][0]*B.v[i][0];
        X.v[1+max_num_motors+i][0] = A.min_throttle.v[i][0]*B.v[i][0];
    }
    return X;
}

// x = H + A*diag(B)*A'
// Note that we only calculate the lower triangle, matrix is expected to be symmetric
MatrixRC<float,AP_MotorsMatrix_Optimal::max_num_motors,AP_MotorsMatrix_Optimal::max_num_motors> AP_MotorsMatrix_Optimal::H_plus_A_mult_b_mult_At(const MatrixRC<float,num_constraints,1>& B) const
{
    MatrixRC<float,max_num_motors,max_num_motors> X;
    for (uint8_t i = 0; i < max_num_motors; i++) {
        for (uint8_t j = 0; j <= i; j++) {
            X.v[i][j] = A.average_throttle.v[i][0] * A.average_throttle.v[j][0] * B.v[0][0];
        }
        X.v[i][i] += A.max_throttle.v[i][0]*A.max_throttle.v[i][0]*B.v[i+1][0] + A.min_throttle.v[i][0]*A.min_throttle.v[i][0]*B.v[i+max_num_motors+1][0];
        for (uint8_t j = 0; j <= i; j++) {
            // MATLAB thinks this is better than adding first, floating point stuff I guess
            X.v[i][j] += H.v[i][j];
        }
    }
    return X;
}

// interior point method quadratic programming solver
// Inspired by: https://github.com/jarredbarber/eigen-QP
// solves min( 0.5*x'Hx + f'x )
// with constraints A'x >= b
void AP_MotorsMatrix_Optimal::interior_point_solve(const MatrixRC<float,max_num_motors,1> &f, const MatrixRC<float,num_constraints,1> &b)
{

    // setup starting points
    x = 0.0;
    z = 1.0;
    s = 1.0;

    // 1/1 = 1
    s_inv = 1.0;

    // compute residuals
    rL = f - A_mult(z);
    rs = s + b;
    rsz = 1.0;
    float mu = num_constraints;

    constexpr float eta = 0.95;
    constexpr float tol = 1e-5;
    constexpr float tol_sq = tol * tol;
    constexpr float tol_nA = tol * num_constraints;
    // limit to 15 iterations
    for (uint8_t k = 0; k < 15; k++) {

        // Pre-decompose to speed up solve
        H_bar = cholesky(H_plus_A_mult_b_mult_At(z.per_element_mult(s_inv)));
        z_rs = z.per_element_mult(rs);

        float alpha;
        for (uint8_t i = 0; i < 2; i++) {
            // centring on fist iteration, correction on second

            // Solve system
            f_bar = rL + A_mult((rsz - z_rs).per_element_mult(s_inv));
            dx = backward_sub_t(H_bar, forward_sub(H_bar,f_bar));
            ds = At_mult(dx) + rs;
            dz = (rsz - z.per_element_mult(ds)).per_element_mult(s_inv);

            // Compute alpha
            alpha = 1.0;
            for (uint8_t j = 0; j < num_constraints; j++) {
                if (dz.v[j][0] > 0) {
                    alpha = MIN(alpha, z.v[j][0]/dz.v[j][0]);
                }
                if (ds.v[j][0] > 0) {
                    alpha = MIN(alpha, s.v[j][0]/ds.v[j][0]);
                }
            }

            if (i == 1) {
                break;
            }

            // affine duality gap
            float mu_a = (z-dz*alpha).dot(s-(ds*alpha));

            // apply centring parameter
            rsz += ds.per_element_mult(dz) - ((mu_a*mu_a*mu_a)/(mu*mu*num_constraints));

        }
        // Update x, z, s
        x -= dx*(alpha*eta);
        z -= dz*(alpha*eta);
        s -= ds*(alpha*eta);

        // Update rhs and mu, checking for convergence
        mu = z.dot(s);
        if (mu < tol_nA) {
            break;
        }
        rs = s - At_mult(x) + b;
        if (rs.dot(rs) < tol_sq) {
            break;
        }
        rL = matrix_multiply(H,x) + f - A_mult(z);
        if (rL.dot(rL) < tol_sq) {
            break;
        }

        // update values for next iteration
        rsz = s.per_element_mult(z);
        s_inv = s.per_element_inv();

    }
}
