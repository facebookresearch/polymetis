// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <franka/lowpass_filter.h>

namespace franka {

double lowpassFilter(double sample_time, double y, double y_last, double cutoff_frequency) {
  if (sample_time < 0 || !std::isfinite(sample_time)) {
    throw std::invalid_argument("lowpass-filter: sample_time is negative, infinite or NaN.");
  }
  if (cutoff_frequency <= 0 || !std::isfinite(cutoff_frequency)) {
    throw std::invalid_argument(
        "lowpass-filter: cutoff_frequency is zero, negative, infinite or NaN.");
  }
  if (!std::isfinite(y) || !std::isfinite(y_last)) {
    throw std::invalid_argument(
        "lowpass-filter: current or past input value of the signal to be filtered is infinite or "
        "NaN.");
  }
  double gain = sample_time / (sample_time + (1.0 / (2.0 * M_PI * cutoff_frequency)));
  return gain * y + (1 - gain) * y_last;
}

std::array<double, 16> cartesianLowpassFilter(double sample_time,
                                              std::array<double, 16> y,
                                              std::array<double, 16> y_last,
                                              double cutoff_frequency) {
  if (sample_time < 0 || !std::isfinite(sample_time)) {
    throw std::invalid_argument(
        "Cartesian lowpass-filter: sample_time is negative, infinite or NaN.");
  }
  if (cutoff_frequency <= 0 || !std::isfinite(cutoff_frequency)) {
    throw std::invalid_argument(
        "Cartesian lowpass-filter: cutoff_frequency is zero, negative, infinite or NaN.");
  }
  for (size_t i = 0; i < y.size(); i++) {
    if (!std::isfinite(y[i]) || !std::isfinite(y_last[i])) {
      throw std::invalid_argument(
          "Cartesian lowpass-filter: current or past input value of the signal to be filtered is "
          "infinite or NaN.");
    }
  }
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(y.data()));
  Eigen::Affine3d transform_last(Eigen::Matrix4d::Map(y_last.data()));
  Eigen::Quaterniond orientation(transform.linear());
  Eigen::Quaterniond orientation_last(transform_last.linear());

  double gain = sample_time / (sample_time + (1.0 / (2.0 * M_PI * cutoff_frequency)));
  transform.translation() =
      gain * transform.translation() + (1.0 - gain) * transform_last.translation();
  orientation = orientation_last.slerp(gain, orientation);

  transform.linear() << orientation.normalized().toRotationMatrix();
  std::array<double, 16> filtered_values{};
  Eigen::Map<Eigen::Matrix4d>(&filtered_values[0], 4, 4) = transform.matrix();

  return filtered_values;
}
}  // namespace franka