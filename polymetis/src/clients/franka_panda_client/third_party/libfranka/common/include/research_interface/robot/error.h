// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

namespace research_interface {
namespace robot {

enum class Error : size_t {
  kJointPositionLimitsViolation,
  kCartesianPositionLimitsViolation,
  kSelfcollisionAvoidanceViolation,
  kJointVelocityViolation,
  kCartesianVelocityViolation,
  kForceControlSafetyViolation,
  kJointReflex,
  kCartesianReflex,
  kMaxGoalPoseDeviationViolation,
  kMaxPathPoseDeviationViolation,
  kCartesianVelocityProfileSafetyViolation,
  kJointPositionMotionGeneratorStartPoseInvalid,
  kJointMotionGeneratorPositionLimitsViolation,
  kJointMotionGeneratorVelocityLimitsViolation,
  kJointMotionGeneratorVelocityDiscontinuity,
  kJointMotionGeneratorAccelerationDiscontinuity,
  kCartesianPositionMotionGeneratorStartPoseInvalid,
  kCartesianMotionGeneratorElbowLimitViolation,
  kCartesianMotionGeneratorVelocityLimitsViolation,
  kCartesianMotionGeneratorVelocityDiscontinuity,
  kCartesianMotionGeneratorAccelerationDiscontinuity,
  kCartesianMotionGeneratorElbowSignInconsistent,
  kCartesianMotionGeneratorStartElbowInvalid,
  kForceControllerDesiredForceToleranceViolation,
  kStartElbowSignInconsistent,
  kCommunicationConstraintsViolation,
  kPowerLimitViolation,
  kCartesianMotionGeneratorJointPositionLimitsViolation,
  kCartesianMotionGeneratorJointVelocityLimitsViolation,
  kCartesianMotionGeneratorJointVelocityDiscontinuity,
  kCartesianMotionGeneratorJointAccelerationDiscontinuity,
  kCartesianPositionMotionGeneratorInvalidFrame,
  kControllerTorqueDiscontinuity,
  kJointP2PInsufficientTorqueForPlanning,
  kTauJRangeViolation,
  kInstabilityDetection,
  kJointMoveInWrongDirection
};

const char* getErrorName(Error error) {
  switch (error) {
    case Error::kCartesianMotionGeneratorAccelerationDiscontinuity:
      return "cartesian_motion_generator_acceleration_discontinuity";
    case Error::kCartesianMotionGeneratorElbowLimitViolation:
      return "cartesian_motion_generator_elbow_limit_violation";
    case Error::kCartesianMotionGeneratorElbowSignInconsistent:
      return "cartesian_motion_generator_elbow_sign_inconsistent";
    case Error::kCartesianMotionGeneratorJointAccelerationDiscontinuity:
      return "cartesian_motion_generator_joint_acceleration_discontinuity";
    case Error::kCartesianMotionGeneratorJointPositionLimitsViolation:
      return "cartesian_motion_generator_joint_position_limits_violation";
    case Error::kCartesianMotionGeneratorJointVelocityDiscontinuity:
      return "cartesian_motion_generator_joint_velocity_discontinuity";
    case Error::kCartesianMotionGeneratorJointVelocityLimitsViolation:
      return "cartesian_motion_generator_joint_velocity_limits_violation";
    case Error::kCartesianMotionGeneratorStartElbowInvalid:
      return "cartesian_motion_generator_start_elbow_invalid";
    case Error::kCartesianMotionGeneratorVelocityDiscontinuity:
      return "cartesian_motion_generator_velocity_discontinuity";
    case Error::kCartesianMotionGeneratorVelocityLimitsViolation:
      return "cartesian_motion_generator_velocity_limits_violation";
    case Error::kCartesianPositionLimitsViolation:
      return "cartesian_position_limits_violation";
    case Error::kCartesianPositionMotionGeneratorInvalidFrame:
      return "cartesian_position_motion_generator_invalid_frame_flag";
    case Error::kCartesianPositionMotionGeneratorStartPoseInvalid:
      return "cartesian_position_motion_generator_start_pose_invalid";
    case Error::kCartesianReflex:
      return "cartesian_reflex";
    case Error::kCartesianVelocityProfileSafetyViolation:
      return "cartesian_velocity_profile_safety_violation";
    case Error::kCartesianVelocityViolation:
      return "cartesian_velocity_violation";
    case Error::kCommunicationConstraintsViolation:
      return "communication_constraints_violation";
    case Error::kControllerTorqueDiscontinuity:
      return "controller_torque_discontinuity";
    case Error::kForceControlSafetyViolation:
      return "force_control_safety_violation";
    case Error::kForceControllerDesiredForceToleranceViolation:
      return "force_controller_desired_force_tolerance_violation";
    case Error::kJointMotionGeneratorAccelerationDiscontinuity:
      return "joint_motion_generator_acceleration_discontinuity";
    case Error::kJointMotionGeneratorPositionLimitsViolation:
      return "joint_motion_generator_position_limits_violation";
    case Error::kJointMotionGeneratorVelocityDiscontinuity:
      return "joint_motion_generator_velocity_discontinuity";
    case Error::kJointMotionGeneratorVelocityLimitsViolation:
      return "joint_motion_generator_velocity_limits_violation";
    case Error::kJointPositionLimitsViolation:
      return "joint_position_limits_violation";
    case Error::kJointPositionMotionGeneratorStartPoseInvalid:
      return "joint_position_motion_generator_start_pose_invalid";
    case Error::kJointReflex:
      return "joint_reflex";
    case Error::kJointVelocityViolation:
      return "joint_velocity_violation";
    case Error::kMaxGoalPoseDeviationViolation:
      return "max_goal_pose_deviation_violation";
    case Error::kMaxPathPoseDeviationViolation:
      return "max_path_pose_deviation_violation";
    case Error::kPowerLimitViolation:
      return "power_limit_violation";
    case Error::kSelfcollisionAvoidanceViolation:
      return "self_collision_avoidance_violation";
    case Error::kStartElbowSignInconsistent:
      return "start_elbow_sign_inconsistent";
    case Error::kJointP2PInsufficientTorqueForPlanning:
      return "joint_p2p_insufficient_torque_for_planning";
    case Error::kTauJRangeViolation:
      return "tau_J_range_violation";
    case Error::kInstabilityDetection:
      return "instability_detected";
    case Error::kJointMoveInWrongDirection:
      return "joint_move_in_wrong_direction";
  }
  throw std::logic_error("Invalid Error given.");
}

}  // namespace robot
}  // namespace research_interface
