// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// --------------------------------------------------------------------------------------

/**
 * @file force_controller.h
 * @brief Outer force control loop for the cascaded servo architecture.
 *
 * Implements a cartesian-space force controller that wraps around the existing
 * position-velocity cascade.  The control law is:
 *
 *   Force_ref  ─[+]─►  [Force PI]  ─► position_correction (dx, dy, dz)
 *               │                             │
 *              [-]                    base_pose + correction = target_pose
 *               │                             │
 *         measured force                [Inverse Kinematics]
 *          (HEX sensor)                       │
 *                                    joint_target_positions
 *                                             │
 *                                    [Position PI] ─► [Velocity PI] ─► Torque
 *
 * When enabled the force controller captures the current end-effector pose as
 * "base pose" and outputs a corrected pose to the existing cascade via IK.
 * The motion controller / path planner is bypassed while force control is active.
 */

#pragma once

#include "pid.h"
#include "hardware/ResenseHEX.h"
#include "utilities/math3d.h"

// ─── ForceTarget ─────────────────────────────────────────────────────────────

struct ForceTarget {
  float fx = 0.0f;  ///< Target force X [mN]
  float fy = 0.0f;  ///< Target force Y [mN]
  float fz = 0.0f;  ///< Target force Z [mN]
};

// ─── ForceController ─────────────────────────────────────────────────────────

class ForceController {
public:
  ForceController();

  // ── Enable / disable ────────────────────────────────────────────────────

  /**
   * @brief Enable or disable force control.
   *
   * On enable the current end-effector pose is latched as "base pose"
   * and all integrators are reset.  On disable the controller outputs are
   * cleared so the system returns to pure position control.
   *
   * @param enable   true  → activate force control
   * @param current_pose  Current EE pose (used only on enable)
   */
  void set_enabled(bool enable, const Pose6DF& current_pose);
  bool is_enabled() const;

  // ── Setpoints ───────────────────────────────────────────────────────────

  void set_target_force(float fx, float fy, float fz);
  const ForceTarget& get_target_force() const;

  // ── Tuning ──────────────────────────────────────────────────────────────

  /**
   * @brief Set PI gains and limits (applied to all three axes equally).
   *
   * @param kp             Proportional gain  [mm / mN]
   * @param ki             Integral gain      [mm / (mN · s)]
   * @param output_limit   Max absolute position correction per axis [mm]
   * @param windup_limit   Max absolute integral contribution [mm]
   */
  void set_gains(float kp, float ki, float output_limit, float windup_limit);

  /**
   * @brief Set lowpass filter time constant for measured force.
   * @param time_constant  Filter time constant [s]
   */
  void set_force_filter_tc(float time_constant);

  /**
   * @brief Set maximum allowable displacement from base pose.
   * @param max_mm  Maximum absolute displacement per axis [mm]
   */
  void set_max_displacement(float max_mm);

  // ── Update (called cyclically from core-0 main loop) ────────────────────

  /**
   * @brief Run one force-control cycle.
   *
   * Filters the measurement, computes the force error, runs the PI
   * controllers, clamps the displacement, and writes the corrected pose
   * into @p target_pose.
   *
   * @param measurement  Current HEX sensor frame
   * @param dt           Time since last call [s]
   * @param[out] target_pose  Corrected EE pose for the position cascade
   * @return true if a valid target_pose was produced
   */
  bool update(const HexFrame& measurement, float dt, Pose6DF& target_pose);

  // ── Reset ───────────────────────────────────────────────────────────────

  void reset();

  // ── Diagnostics ─────────────────────────────────────────────────────────

  void get_measured_force(float& fx, float& fy, float& fz) const;
  void get_position_correction(float& dx, float& dy, float& dz) const;

private:
  bool enabled;

  ForceTarget target;

  // Filtered measurements
  float filtered_fx, filtered_fy, filtered_fz;

  // Base pose (latched when force control was enabled)
  Pose6DF base_pose;

  // Current position correction from PI output [mm]
  float correction_x, correction_y, correction_z;

  // Safety: max displacement from base pose per axis [mm]
  float max_displacement;

  // PI controllers – one per cartesian force axis
  PIDController pid_x;
  PIDController pid_y;
  PIDController pid_z;

  // Lowpass filters for force measurements
  LowpassFilter filter_x;
  LowpassFilter filter_y;
  LowpassFilter filter_z;
};
