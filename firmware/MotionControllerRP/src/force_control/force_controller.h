// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  Force Control Module
// --------------------------------------------------------------------------------------

#pragma once

#include "servo_control/pid.h"
#include "utilities/math_constants.h"
#include "motion_control/path_segment.h"  // for NUM_JOINTS

//--- ForceControllerConfig -------------------------------------------------------------

struct ForceControllerConfig {
  // PI gains (applied to all axes unless overridden per-axis)
  float kp = 0.001f;
  float ki = 0.01f;
  float output_limit = Constants::PI_F * 0.45f;  // max torque command (field angle offset in rad)
  float windup_limit = Constants::PI_F * 0.45f;  // anti-windup limit

  // Safety: encoder position soft limits per axis (motor position in radians)
  float pos_limit_min[NUM_JOINTS] = {-1.5f, -1.5f, -1.5f};
  float pos_limit_max[NUM_JOINTS] = { 1.5f,  1.5f,  1.5f};
};

//--- ForceControllerState --------------------------------------------------------------

struct ForceControllerState {
  float target_force[NUM_JOINTS]   = {0.0f, 0.0f, 0.0f};
  float measured_force[NUM_JOINTS] = {0.0f, 0.0f, 0.0f};
  float force_error[NUM_JOINTS]    = {0.0f, 0.0f, 0.0f};
  float torque_output[NUM_JOINTS]  = {0.0f, 0.0f, 0.0f};
  float motor_position[NUM_JOINTS] = {0.0f, 0.0f, 0.0f};

  bool  axis_safety_triggered[NUM_JOINTS] = {false, false, false};
  bool  active    = false;
  bool  sensor_ok = false;
};

//--- ForceController -------------------------------------------------------------------

/// Pure force control with 3 independent PI loops.
/// Motor 0→X, Motor 1→Y, Motor 2→Z (direct mapping, no kinematics).
/// Encoder data is used only for safety monitoring (soft position limits),
/// NOT for feedback within the control loop.
class ForceController {
public:
  ForceController();

  // --- Configuration ---

  /// Set PI parameters for ALL axes.
  void set_pi_parameters(float kp, float ki, float output_limit, float windup_limit);

  /// Set target forces in mN (or sensor-native units) for X, Y, Z.
  void set_target_force(float fx, float fy, float fz);

  /// Set symmetric safety position limits for one axis: ±limit.
  void set_safety_limits(int axis, float limit);

  /// Set asymmetric safety position limits for one axis.
  void set_safety_limits(int axis, float min_pos, float max_pos);

  // --- Control loop ---

  /// Main update. Call at the force-control rate (~1-2 kHz).
  ///   measured_forces:  [fx, fy, fz] from HEX sensor
  ///   motor_positions:  current motor positions from encoders (for safety only)
  ///   dt:               time step in seconds
  ///   torque_outputs:   (out) computed torque commands for each motor
  /// Returns true if outputs are valid (control active & sensor ok & no safety trip).
  bool update(const float measured_forces[NUM_JOINTS],
              const float motor_positions[NUM_JOINTS],
              float dt,
              float torque_outputs[NUM_JOINTS]);

  /// Enable force control (resets integrators).
  void enable();

  /// Disable force control (zeroes outputs).
  void disable();

  /// Reset integrators without changing enable state.
  void reset();

  // --- Accessors ---

  bool is_active() const { return state.active; }
  const ForceControllerState& get_state() const { return state; }
  const ForceControllerConfig& get_config() const { return config; }

private:
  /// Check whether motor_pos is within configured soft limits for the axis.
  bool check_safety(int axis, float motor_pos);

  PIDController pi_controllers[NUM_JOINTS];
  ForceControllerConfig config;
  ForceControllerState  state;
};
