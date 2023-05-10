#pragma once

#include "pjpo_curvature/vehicle_param.h"

#include "pjpo_curvature/math/vec2d.h"
#include "pjpo_curvature/math/polygon2d.h"

#include <mutex>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include "color.h"

#include "geometry_msgs/PoseArray.h"  
#include "pjpo_curvature/FrenetPath.h"

namespace pjpo_curvature {
namespace visualization {

using math::Vec2d;
using math::Polygon2d;

using Vector = std::vector<double>;

void Init(ros::NodeHandle &node, const std::string &frame, const std::string &topic);

void Plot(const Vector &xs, const Vector &ys, double width = 0.1, Color color = Color(1, 1, 1),
          int id = -1, const std::string &ns = "");

void Plot(const Vector &xs, const Vector &ys, double width = 0.1, const std::vector<Color> &color = {},
          int id = -1, const std::string &ns = "");

void PlotPolygon(const Vector &xs, const Vector &ys, double width = 0.1, Color color = Color::White,
                 int id = -1, const std::string &ns = "");

void PlotPolygon(const Polygon2d &polygon, double width = 0.1, Color color = Color::White,
                 int id = -1, const std::string &ns = "");

void PlotTrajectory(const Vector &xs, const Vector &ys, const Vector &vs, double max_velocity = 10.0,
                    double width = 0.1, const Color &color = Color::Blue,
                    int id = -1, const std::string &ns = "");

void PlotPoints(const Vector &xs, const Vector &ys, double width = 0.1, const Color &color = Color::White,
                int id = -1, const std::string &ns = "");

void Trigger();

void SLdraw(const geometry_msgs::PoseArray & l_, const geometry_msgs::PoseArray & dl_, const geometry_msgs::PoseArray & ddl_, const geometry_msgs::PoseArray & dddl_);

void Clear();

void PubKappaInfo(const geometry_msgs::PoseArray & kappa_info);

void PubSLPath(const pjpo_curvature::FrenetPath & path);

}

}