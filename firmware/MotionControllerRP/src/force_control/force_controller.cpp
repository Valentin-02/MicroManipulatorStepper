// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  Force Control Module
// --------------------------------------------------------------------------------------

#include "force_controller.h"
#include <algorithm>

//*** CLASS *****************************************************************************

ForceController::ForceController() {
  // Apply default PI parameters to all axes
  for (int i = 0; i < NUM_JOINTS; i++) {
    pi_controllers[i].set_parameter(
      config.kp, config.ki, 0.0f,         // kP, kI, kD=0 (PI only)
      config.output_limit, config.windup_limit
    );
  }
}

//--- Configuration ---------------------------------------------------------------------

void ForceController::set_pi_parameters(float kp, float ki, float output_limit, float windup_limit) {
  config.kp = kp;
  config.ki = ki;
  config.output_limit = output_limit;
  config.windup_limit = windup_limit;

  for (int i = 0; i < NUM_JOINTS; i++) {
    pi_controllers[i].set_parameter(kp, ki, 0.0f, output_limit, windup_limit);
  }
}

void ForceController::set_target_force(float fx, float fy, float fz) {
  state.target_force[0] = fx;
  state.target_force[1] = fy;
  state.target_force[2] = fz;
}

void ForceController::set_safety_limits(int axis, float limit) {
  if (axis < 0 || axis >= NUM_JOINTS) return;
  config.pos_limit_min[axis] = -limit;
  config.pos_limit_max[axis] =  limit;
}

void ForceController::set_safety_limits(int axis, float min_pos, float max_pos) {
  if (axis < 0 || axis >= NUM_JOINTS) return;
  config.pos_limit_min[axis] = min_pos;
  config.pos_limit_max[axis] = max_pos;
}

//--- Control loop ----------------------------------------------------------------------

bool ForceController::update(const float measured_forces[NUM_JOINTS],
                             const float motor_positions[NUM_JOINTS],
                             float dt,
                             float torque_outputs[NUM_JOINTS])
{
  if (!state.active) {
    for (int i = 0; i < NUM_JOINTS; i++) torque_outputs[i] = 0.0f;
    return false;
  }

  float one_over_dt = (dt > 1e-9f) ? (1.0f / dt) : 0.0f;
  bool all_ok = true;

  for (int i = 0; i < NUM_JOINTS; i++) {
    // Store measured values for state reporting
    state.measured_force[i] = measured_forces[i];
    state.motor_position[i] = motor_positions[i];

    // --- Safety check (encoder-based soft limits) ---
    if (!check_safety(i, motor_positions[i])) {
      // Safety triggered: zero torque for this axis
      state.axis_safety_triggered[i] = true;
      state.torque_output[i] = 0.0f;
      torque_outputs[i] = 0.0f;
      pi_controllers[i].reset();
      all_ok = false;
      continue;
    }
    state.axis_safety_triggered[i] = false;

    // --- PI force control ---
    float error = state.target_force[i] - measured_forces[i];
    state.force_error[i] = error;

    float torque = pi_controllers[i].compute(error, dt, one_over_dt);
    state.torque_output[i] = torque;
    torque_outputs[i] = torque;
  }

  return all_ok;
}

//--- Enable / Disable ------------------------------------------------------------------

void ForceController::enable() {
  reset();
  state.active = true;
}

void ForceController::disable() {
  state.active = false;
  for (int i = 0; i < NUM_JOINTS; i++) {
    state.torque_output[i] = 0.0f;
    state.force_error[i]   = 0.0f;
    state.axis_safety_triggered[i] = false;
  }
  reset();
}

void ForceController::reset() {
  for (int i = 0; i < NUM_JOINTS; i++) {
    pi_controllers[i].reset();
  }
}

//--- Safety check ----------------------------------------------------------------------

bool ForceController::check_safety(int axis, float motor_pos) {
  return (motor_pos >= config.pos_limit_min[axis]) &&
         (motor_pos <= config.pos_limit_max[axis]);
}
