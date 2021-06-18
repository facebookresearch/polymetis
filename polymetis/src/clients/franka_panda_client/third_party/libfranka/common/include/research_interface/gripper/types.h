// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace research_interface {
namespace gripper {

#pragma pack(push, 1)

using Version = uint16_t;

constexpr Version kVersion = 3;
constexpr uint16_t kCommandPort = 1338;

enum class Command : uint16_t { kConnect, kHoming, kGrasp, kMove, kStop };

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
  static_assert(
      std::is_same<typename std::underlying_type<decltype(status)>::type, uint16_t>::value,
      "Status must be of type uint16_t.");
  static_assert(static_cast<uint16_t>(decltype(status)::kSuccess) == 0,
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

  enum class Status : uint16_t { kSuccess, kFail, kUnsuccessful, kAborted };

  using Header = CommandHeader;
  using Request = RequestBase<T>;
  using Response = ResponseBase<T>;
  template <typename P>
  using Message = CommandMessage<P>;
};

struct Connect : CommandBase<Connect, Command::kConnect> {
  enum class Status : uint16_t { kSuccess, kIncompatibleLibraryVersion };

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

struct Homing : public CommandBase<Homing, Command::kHoming> {};

struct Grasp : public CommandBase<Grasp, Command::kGrasp> {
  struct GraspEpsilon {
    constexpr GraspEpsilon(double inner, double outer) : inner(inner), outer(outer) {}
    const double inner;
    const double outer;
  };
  struct Request : public RequestBase<Grasp> {
    Request(double width, GraspEpsilon epsilon, double speed, double force)
        : width(width), epsilon(epsilon), speed(speed), force(force) {}

    const double width;
    const GraspEpsilon epsilon;
    const double speed;
    const double force;
  };
};

struct Move : public CommandBase<Move, Command::kMove> {
  struct Request : public RequestBase<Move> {
    Request(double width, double speed) : width(width), speed(speed) {}

    const double width;
    const double speed;
  };
};

struct Stop : public CommandBase<Stop, Command::kStop> {};

struct GripperState {
  uint32_t message_id;
  double width;
  double max_width;
  bool is_grasped;
  uint16_t temperature;
};

#pragma pack(pop)

}  // namespace gripper
}  // namespace research_interface
