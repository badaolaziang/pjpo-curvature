#pragma once

#include <utility>
#include <vector>
#include <cassert>

#include "pjpo_curvature/math/vec2d.h"

namespace pjpo_curvature {

struct TrajectoryPoint {
  double s = 0.0;
  //delta_s
  double ds = 0.0;
  double l = 0.0;
  double dl = 0.0;
  double ddl = 0.0;

  double x = 0.0;
  double y = 0.0;
  double theta = 0.0; 
  double kappa = 0.0;
  double dkappa = 0.0;
  double velocity = 0.0;

  double left_bound = 0.0;
  double right_bound = 0.0;
};

typedef std::vector<TrajectoryPoint> Trajectory;

using math::Vec2d;

/**
 * Discretized Trajectory
 */
class DiscretizedTrajectory {
public:
  typedef std::vector<TrajectoryPoint> DataType;
  typedef unsigned long size_t;

  DiscretizedTrajectory() = default;

  DiscretizedTrajectory(const DiscretizedTrajectory &rhs, size_t begin, size_t end = -1);

  explicit DiscretizedTrajectory(std::vector<TrajectoryPoint> points) : data_(std::move(points)) {}

  inline const DataType &data() const { return data_; }

  DataType::const_iterator QueryLowerBoundStationPoint(double station) const;

  DataType::const_iterator QueryNearestPoint(const Vec2d &point, double *out_distance = nullptr) const;

  TrajectoryPoint EvaluateStation(double station) const;

  Vec2d GetProjection(const Vec2d &xy) const;

  Vec2d GetCartesian(double station, double lateral) const;


// protected:
  std::vector<TrajectoryPoint> data_;
};

}
