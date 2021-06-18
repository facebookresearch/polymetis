// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace research_interface {
namespace robot {

#pragma pack(push, 1)

using Version = uint16_t;

constexpr Version kVersion = 4;
constexpr uint16_t kCommandPort = 1337;

enum class Command : uint32_t {
  kConnect,
  kMove,
  kStopMove,
  kGetCartesianLimit,
  kSetCollisionBehavior,
  kSetJointImpedance,
  kSetCartesianImpedance,
  kSetGuidingMode,
  kSetEEToK,
  kSetNEToEE,
  kSetLoad,
  kSetFilters,
  kAutomaticErrorRecovery,
  kLoadModelLibrary
};

struct CommandHeader {
  CommandHeader() = default;
  CommandHeader(Command command, uint32_t command_id, uint32_t size)
      : command(command), command_id(command_id), size(size) {}

  Command command;
  uint32_t command_id;
  uint32_t size;
};

template <typename T>
struct RequestBase {};

template <typename T>
struct ResponseBase {
  ResponseBase(typename T::Status status) : status(status) {}

  const typename T::Status status;

  static_assert(std::is_enum<decltype(status)>::value, "Status must be an enum.");
  static_assert(std::is_same<typename std::underlying_type<decltype(status)>::type, uint8_t>::value,
                "Status must be of type uint8_t.");
  static_assert(static_cast<uint32_t>(decltype(status)::kSuccess) == 0,
                "Status must define kSuccess with value of 0.");
};

template <typename T>
struct CommandMessage {
  CommandMessage() = default;
  CommandMessage(const CommandHeader& header, const T& instance) : header(header) {
    std::memcpy(payload.data(), &instance, payload.size());
  }

  T getInstance() const noexcept { return *reinterpret_cast<const T*>(payload.data()); }

  CommandHeader header;
  std::array<uint8_t, sizeof(T)> payload;
};

template <typename T>
struct CommandMessage<RequestBase<T>> {
  CommandMessage() = default;
  CommandMessage(const CommandHeader& header, const RequestBase<T>&) : header(header) {}

  RequestBase<T> getInstance() const noexcept { return RequestBase<T>(); }

  CommandHeader header;
};

template <typename T, Command C>
struct CommandBase {
  CommandBase() = delete;

  static constexpr Command kCommand = C;

  enum class Status : uint8_t { kSuccess, kCommandNotPossibleRejected };

  using Header = CommandHeader;
  using Request = RequestBase<T>;
  using Response = ResponseBase<T>;
  template <typename P>
  using Message = CommandMessage<P>;
};

template <typename T, Command C>
struct GetterSetterCommandBase : CommandBase<T, C> {
  enum class Status : uint8_t { kSuccess, kCommandNotPossibleRejected, kInvalidArgumentRejected };
};

struct Connect : CommandBase<Connect, Command::kConnect> {
  enum class Status : uint8_t { kSuccess, kIncompatibleLibraryVersion };

  struct Request : public RequestBase<Connect> {
    Request(uint16_t udp_port) : version(kVersion), udp_port(udp_port) {}

    const Version version;
    const uint16_t udp_port;
  };

  struct Response : public ResponseBase<Connect> {
    Response(Status status) : ResponseBase(status), version(kVersion) {}

    const Version version;
  };
};

struct Move : public CommandBase<Move, Command::kMove> {
  enum class ControllerMode : uint32_t {
    kJointImpedance,
    kCartesianImpedance,
    kExternalController
  };

  enum class MotionGeneratorMode : uint32_t {
    kJointPosition,
    kJointVelocity,
    kCartesianPosition,
    kCartesianVelocity
  };

  enum class Status : uint8_t {
    kSuccess,
    kMotionStarted,
    kPreempted,
    kCommandNotPossibleRejected,
    kStartAtSingularPoseRejected,
    kInvalidArgumentRejected,
    kReflexAborted,
    kEmergencyAborted,
    kInputErrorAborted,
    kAborted
  };

  struct Deviation {
    constexpr Deviation(double translation, double rotation, double elbow)
        : translation(translation), rotation(rotation), elbow(elbow) {}
    const double translation;
    const double rotation;
    const double elbow;
  };

  struct Request : public RequestBase<Move> {
    Request(ControllerMode controller_mode,
            MotionGeneratorMode motion_generator_mode,
            const Deviation& maximum_path_deviation,
            const Deviation& maximum_goal_pose_deviation)
        : controller_mode(controller_mode),
          motion_generator_mode(motion_generator_mode),
          maximum_path_deviation(maximum_path_deviation),
          maximum_goal_pose_deviation(maximum_goal_pose_deviation) {}

    const ControllerMode controller_mode;
    const MotionGeneratorMode motion_generator_mode;
    const Deviation maximum_path_deviation;
    const Deviation maximum_goal_pose_deviation;
  };
};

struct StopMove : public CommandBase<StopMove, Command::kStopMove> {
  enum class Status : uint8_t {
    kSuccess,
    kCommandNotPossibleRejected,
    kEmergencyAborted,
    kReflexAborted,
    kAborted
  };
};

struct GetCartesianLimit
    : public GetterSetterCommandBase<GetCartesianLimit, Command::kGetCartesianLimit> {
  struct Request : public RequestBase<GetCartesianLimit> {
    Request(int32_t id) : id(id) {}

    const int32_t id;
  };

  struct Response : public ResponseBase<GetCartesianLimit> {
    Response(Status status,
             const std::array<double, 3>& object_world_size,
             const std::array<double, 16>& object_frame,
             bool object_activation)
        : ResponseBase(status),
          object_world_size(object_world_size),
          object_frame(object_frame),
          object_activation(object_activation) {}
    Response(Status status) : Response(status, {}, {}, false) {}

    const std::array<double, 3> object_world_size;
    const std::array<double, 16> object_frame;
    const bool object_activation;
  };
};

struct SetCollisionBehavior
    : public GetterSetterCommandBase<SetCollisionBehavior, Command::kSetCollisionBehavior> {
  struct Request : public RequestBase<SetCollisionBehavior> {
    Request(const std::array<double, 7>& lower_torque_thresholds_acceleration,
            const std::array<double, 7>& upper_torque_thresholds_acceleration,
            const std::array<double, 7>& lower_torque_thresholds_nominal,
            const std::array<double, 7>& upper_torque_thresholds_nominal,
            const std::array<double, 6>& lower_force_thresholds_acceleration,
            const std::array<double, 6>& upper_force_thresholds_acceleration,
            const std::array<double, 6>& lower_force_thresholds_nominal,
            const std::array<double, 6>& upper_force_thresholds_nominal)
        : lower_torque_thresholds_acceleration(lower_torque_thresholds_acceleration),
          upper_torque_thresholds_acceleration(upper_torque_thresholds_acceleration),
          lower_torque_thresholds_nominal(lower_torque_thresholds_nominal),
          upper_torque_thresholds_nominal(upper_torque_thresholds_nominal),
          lower_force_thresholds_acceleration(lower_force_thresholds_acceleration),
          upper_force_thresholds_acceleration(upper_force_thresholds_acceleration),
          lower_force_thresholds_nominal(lower_force_thresholds_nominal),
          upper_force_thresholds_nominal(upper_force_thresholds_nominal) {}

    const std::array<double, 7> lower_torque_thresholds_acceleration;
    const std::array<double, 7> upper_torque_thresholds_acceleration;

    const std::array<double, 7> lower_torque_thresholds_nominal;
    const std::array<double, 7> upper_torque_thresholds_nominal;

    const std::array<double, 6> lower_force_thresholds_acceleration;
    const std::array<double, 6> upper_force_thresholds_acceleration;

    const std::array<double, 6> lower_force_thresholds_nominal;
    const std::array<double, 6> upper_force_thresholds_nominal;
  };
};

struct SetJointImpedance
    : public GetterSetterCommandBase<SetJointImpedance, Command::kSetJointImpedance> {
  struct Request : public RequestBase<SetJointImpedance> {
    Request(const std::array<double, 7>& K_theta) : K_theta(K_theta) {}

    const std::array<double, 7> K_theta;
  };
};

struct SetCartesianImpedance
    : public GetterSetterCommandBase<SetCartesianImpedance, Command::kSetCartesianImpedance> {
  struct Request : public RequestBase<SetCartesianImpedance> {
    Request(const std::array<double, 6>& K_x) : K_x(K_x) {}

    const std::array<double, 6> K_x;
  };
};

struct SetGuidingMode : public GetterSetterCommandBase<SetGuidingMode, Command::kSetGuidingMode> {
  struct Request : public RequestBase<SetGuidingMode> {
    Request(const std::array<bool, 6>& guiding_mode, bool nullspace)
        : guiding_mode(guiding_mode), nullspace(nullspace) {}

    const std::array<bool, 6> guiding_mode;
    const bool nullspace;
  };
};

struct SetEEToK : public GetterSetterCommandBase<SetEEToK, Command::kSetEEToK> {
  struct Request : public RequestBase<SetEEToK> {
    Request(const std::array<double, 16>& EE_T_K) : EE_T_K(EE_T_K) {}

    const std::array<double, 16> EE_T_K;
  };
};

struct SetNEToEE : public GetterSetterCommandBase<SetNEToEE, Command::kSetNEToEE> {
  struct Request : public RequestBase<SetNEToEE> {
    Request(const std::array<double, 16>& NE_T_EE) : NE_T_EE(NE_T_EE) {}

    const std::array<double, 16> NE_T_EE;
  };
};

struct SetLoad : public GetterSetterCommandBase<SetLoad, Command::kSetLoad> {
  struct Request : public RequestBase<SetLoad> {
    Request(double m_load,
            const std::array<double, 3>& F_x_Cload,
            const std::array<double, 9>& I_load)
        : m_load(m_load), F_x_Cload(F_x_Cload), I_load(I_load) {}

    const double m_load;
    const std::array<double, 3> F_x_Cload;
    const std::array<double, 9> I_load;
  };
};

struct SetFilters : public GetterSetterCommandBase<SetFilters, Command::kSetFilters> {
  struct Request : public RequestBase<SetFilters> {
    Request(double joint_position_filter_frequency,
            double joint_velocity_filter_frequency,
            double cartesian_position_filter_frequency,
            double cartesian_velocity_filter_frequency,
            double controller_filter_frequency)
        : joint_position_filter_frequency(joint_position_filter_frequency),
          joint_velocity_filter_frequency(joint_velocity_filter_frequency),
          cartesian_position_filter_frequency(cartesian_position_filter_frequency),
          cartesian_velocity_filter_frequency(cartesian_velocity_filter_frequency),
          controller_filter_frequency(controller_filter_frequency) {}

    const double joint_position_filter_frequency;
    const double joint_velocity_filter_frequency;
    const double cartesian_position_filter_frequency;
    const double cartesian_velocity_filter_frequency;
    const double controller_filter_frequency;
  };
};

struct AutomaticErrorRecovery
    : public CommandBase<AutomaticErrorRecovery, Command::kAutomaticErrorRecovery> {
  enum class Status : uint8_t {
    kSuccess,
    kCommandNotPossibleRejected,
    kManualErrorRecoveryRequiredRejected,
    kReflexAborted,
    kEmergencyAborted,
    kAborted
  };
};

struct LoadModelLibrary : public CommandBase<LoadModelLibrary, Command::kLoadModelLibrary> {
  enum class Status : uint8_t { kSuccess, kError };

  enum class Architecture : uint8_t { kX64, kX86, kARM, kARM64 };

  enum class System : uint8_t { kLinux, kWindows };

  struct Request : public RequestBase<LoadModelLibrary> {
    Request(Architecture architecture, System system)
        : architecture(architecture), system(system) {}

    const Architecture architecture;
    const System system;
  };
};

#pragma pack(pop)

}  // namespace robot
}  // namespace research_interface
