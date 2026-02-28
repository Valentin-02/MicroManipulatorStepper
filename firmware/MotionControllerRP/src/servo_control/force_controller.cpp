// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// --------------------------------------------------------------------------------------

/**
 * @file force_controller.cpp
 * @brief Implementation of the outer cartesian force control loop.
 *
 * The controller runs three independent PI loops (one per cartesian axis)
 * that map force error [mN] to a position correction [mm].  The corrected
 * pose is built from a latched base pose plus the PI output and handed to
 * the existing position-velocity cascade via the inverse-kinematics path.
 */

#include "force_controller.h"
#include "utilities/logging.h"
#include <algorithm>
#include <cmath>

// ─── Constructor ─────────────────────────────────────────────────────────────

ForceController::ForceController()
    : enabled(false)
    , filtered_fx(0.0f), filtered_fy(0.0f), filtered_fz(0.0f)
    , correction_x(0.0f), correction_y(0.0f), correction_z(0.0f)
    , max_displacement(1.0f)  // safety default: 1 mm max displacement
{
  // Conservative default gains
  //   kP = 0.001  mm/mN   → 1 µm per mN of force error
  //   kI = 0.01   mm/(mN·s) → eliminates steady-state error
  float kp = 0.001f;
  float ki = 0.01f;
  float output_limit = 1.0f;   // mm
  float windup_limit = 0.5f;   // mm

  pid_x.set_parameter(kp, ki, 0.0f, output_limit, windup_limit);
  pid_y.set_parameter(kp, ki, 0.0f, output_limit, windup_limit);
  pid_z.set_parameter(kp, ki, 0.0f, output_limit, windup_limit);

  // Force measurement filter: 5 ms time constant
  filter_x.set_time_constant(0.005f);
  filter_y.set_time_constant(0.005f);
  filter_z.set_time_constant(0.005f);

  target = {0.0f, 0.0f, 0.0f};
}

// ─── Enable / Disable ───────────────────────────────────────────────────────

void ForceController::set_enabled(bool enable, const Pose6DF& current_pose) {
  if (enable && !enabled) {
    // Transitioning OFF → ON: latch base pose, reset state
    base_pose = current_pose;
    reset();
    LOG_INFO("Force controller enabled  (base pose: X=%.3f Y=%.3f Z=%.3f)",
             base_pose.translation.x,
             base_pose.translation.y,
             base_pose.translation.z);
  }
  if (!enable && enabled) {
    LOG_INFO("Force controller disabled");
  }
  enabled = enable;
}

bool ForceController::is_enabled() const {
  return enabled;
}

// ─── Setpoints ──────────────────────────────────────────────────────────────

void ForceController::set_target_force(float fx, float fy, float fz) {
  target.fx = fx;
  target.fy = fy;
  target.fz = fz;
}

const ForceTarget& ForceController::get_target_force() const {
  return target;
}

// ─── Tuning ─────────────────────────────────────────────────────────────────

void ForceController::set_gains(float kp, float ki,
                                float output_limit, float windup_limit)
{
  pid_x.set_parameter(kp, ki, 0.0f, output_limit, windup_limit);
  pid_y.set_parameter(kp, ki, 0.0f, output_limit, windup_limit);
  pid_z.set_parameter(kp, ki, 0.0f, output_limit, windup_limit);
}

void ForceController::set_force_filter_tc(float time_constant) {
  filter_x.set_time_constant(time_constant);
  filter_y.set_time_constant(time_constant);
  filter_z.set_time_constant(time_constant);
}

void ForceController::set_max_displacement(float max_mm) {
  max_displacement = fabsf(max_mm);
}

// ─── Cyclic update ──────────────────────────────────────────────────────────

bool ForceController::update(const HexFrame& measurement, float dt,
                             Pose6DF& target_pose)
{
  if (!enabled)
    return false;

  // Guard against degenerate dt
  if (dt <= 0.0f || dt > 0.1f)
    return false;

  float one_over_dt = 1.0f / dt;

  // ── 1. Filter the measured forces ──────────────────────────────────────
  filtered_fx = filter_x.update(measurement.fx, dt);
  filtered_fy = filter_y.update(measurement.fy, dt);
  filtered_fz = filter_z.update(measurement.fz, dt);

  // ── 2. Compute force error (positive error → need more force) ─────────
  float error_fx = target.fx - filtered_fx;
  float error_fy = target.fy - filtered_fy;
  float error_fz = target.fz - filtered_fz;

  // ── 3. PI controllers → position correction [mm] ─────────────────────
  correction_x = pid_x.compute(error_fx, dt, one_over_dt);
  correction_y = pid_y.compute(error_fy, dt, one_over_dt);
  correction_z = pid_z.compute(error_fz, dt, one_over_dt);

  // ── 4. Clamp displacement for safety ──────────────────────────────────
  correction_x = std::clamp(correction_x, -max_displacement, max_displacement);
  correction_y = std::clamp(correction_y, -max_displacement, max_displacement);
  correction_z = std::clamp(correction_z, -max_displacement, max_displacement);

  // ── 5. Build corrected pose ───────────────────────────────────────────
  target_pose = base_pose;
  target_pose.translation.x += correction_x;
  target_pose.translation.y += correction_y;
  target_pose.translation.z += correction_z;
  // Rotation remains unchanged (pure translational force control)

  return true;
}

// ─── Reset ──────────────────────────────────────────────────────────────────

void ForceController::reset() {
  pid_x.reset();
  pid_y.reset();
  pid_z.reset();

  filter_x.reset(0.0f);
  filter_y.reset(0.0f);
  filter_z.reset(0.0f);

  filtered_fx = filtered_fy = filtered_fz = 0.0f;
  correction_x = correction_y = correction_z = 0.0f;
}

// ─── Diagnostics ────────────────────────────────────────────────────────────

void ForceController::get_measured_force(float& fx, float& fy, float& fz) const {
  fx = filtered_fx;
  fy = filtered_fy;
  fz = filtered_fz;
}

void ForceController::get_position_correction(float& dx, float& dy, float& dz) const {
  dx = correction_x;
  dy = correction_y;
  dz = correction_z;
}
