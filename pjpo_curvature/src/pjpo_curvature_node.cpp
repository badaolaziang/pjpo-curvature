#include <ros/ros.h>

#include "geometry_msgs/PoseStamped.h"
#include "pjpo_curvature/CenterLine.h"
#include "pjpo_curvature/Obstacles.h"
#include "pjpo_curvature/DynamicObstacles.h"
#include "pjpo_curvature/pjpo_curvature.h"

#include "pjpo_curvature/visualization/plot.h"

using namespace pjpo_curvature;

class PJPOCurvatureNode {
public:
  explicit PJPOCurvatureNode(const ros::NodeHandle &nh) : nh_(nh) {
    env_ = std::make_shared<Environment>(config_);
    planner_ = std::make_shared<PJPOCurvature>(config_, env_);

    center_line_subscriber_ = nh_.subscribe("/center_line", 1, &PJPOCurvatureNode::CenterLineCallback, this);
    obstacles_subscriber_ = nh_.subscribe("/obstacles", 1, &PJPOCurvatureNode::ObstaclesCallback, this);
    dynamic_obstacles_subscriber_ = nh_.subscribe("/dynamic_obstacles", 1,
                                                  &PJPOCurvatureNode::DynamicObstaclesCallback, this);

    goal_subscriber_ = nh_.subscribe("/move_base_simple/goal", 1, &PJPOCurvatureNode::PlanCallback, this);

    state_.x = 0.0;
    state_.y = 0.0;
    state_.theta = 0.0;
    state_.v = 5.0;
    state_.phi = 0.0;
    state_.a = 0.0;
    state_.omega = 0.0;
    ros::param::get("w_l",planner_->w_l);
    ros::param::get("w_dl",planner_->w_dl);
    ros::param::get("w_ddl",planner_->w_ddl);
    ros::param::get("w_dddl",planner_->w_dddl);
    ros::param::get("w_obs",planner_->w_obs);
    ros::param::get("w_kappa_2",planner_->w_kappa_2);
    ros::param::get("w_dkappa",planner_->w_dkappa);
    ros::param::get("lat_speed_limit",planner_->lat_speed_limit);
    ros::param::get("lat_acc_limit",planner_->lat_acc_limit);
    ros::param::get("lat_jerk_limit",planner_->lat_jerk_limit);
    ros::param::get("kappa_limit",planner_->kappa_limit);
    ros::param::get("dkappa_limit",planner_->dkappa_limit);
    ros::param::get("smooth_bounding_dist",planner_->smooth_bounding_dist);
    ros::param::get("w_smooth",planner_->w_smooth);
    ros::param::get("starting_l",planner_->starting_l);
    ros::param::get("car_info_L",planner_->car_info_L);
  }

  void CenterLineCallback(const CenterLineConstPtr &msg) {
    Trajectory data;
    double ds_offset = 0;
    double ds_0 = msg->points[1].s - msg->points[0].s;
    for (auto &pt: msg->points) {
      TrajectoryPoint tp;
      if(!data.empty() && (hypot(data.back().x - pt.x, data.back().y - pt.y) < 0.2 * (pt.s - ds_offset - data.back().s)) ){
          std::cout << "test: " << hypot(data.back().x - pt.x, data.back().y - pt.y) << " " << pt.s - ds_offset - data.back().s << std::endl;
          ds_offset += ds_0;
          continue;
      }
      tp.s = pt.s - ds_offset;
      tp.x = pt.x;
      tp.y = pt.y;
      tp.theta = pt.theta;
      tp.kappa = pt.kappa;
      tp.left_bound = pt.left_bound;
      tp.right_bound = pt.right_bound;
      data.push_back(tp);
    }

    env_->SetReference(DiscretizedTrajectory(data));
    env_->Visualize();
  }

  void ObstaclesCallback(const ObstaclesConstPtr &msg) {
    env_->obstacles().clear();
    for (auto &obstacle: msg->obstacles) {
      std::vector<math::Vec2d> points;
      for (auto &pt: obstacle.points) {
        points.emplace_back(pt.x, pt.y);
      }
      env_->obstacles().emplace_back(points);
    }
    env_->Visualize();
  }

  void DynamicObstaclesCallback(const DynamicObstaclesConstPtr &msg) {
    env_->dynamic_obstacles().clear();
    for (auto &obstacle: msg->obstacles) {
      Environment::DynamicObstacle dynamic_obstacle;

      for (auto &tp: obstacle.trajectory) {
        math::Pose coord(tp.x, tp.y, tp.theta);
        std::vector<math::Vec2d> points;
        for (auto &pt: obstacle.polygon.points) {
          points.push_back(coord.transform({pt.x, pt.y, 0.0}));
        }
        math::Polygon2d polygon(points);

        dynamic_obstacle.emplace_back(tp.time, polygon);
      }

      env_->dynamic_obstacles().push_back(dynamic_obstacle);
    }
    env_->Visualize();
  }

  void PlanCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
    // env_->obstacles().clear();
    // std::vector<std::pair<int, double>> obs_sl = {{20, 1}, {40, -1}};
    // for (int i = 0; i < obs_sl.size(); ++i) {
    //   std::vector<math::Vec2d> points;
    //   std::vector<double> dx = {2,2,-2,-2}, dy = {1, -1, -1, 1};
    //   for(int j = 0; j < 4; ++i){
    //     points.emplace_back(env_->reference().data_[i].x + dx[j]*sin(env_->reference().data_[i].theta),
    //                      env_->reference().data_[i].y + dy[j]*sin(env_->reference().data_[i].theta));
    //   }
    //   env_->obstacles().emplace_back(points);
    // }
    // env_->Visualize();
    DiscretizedTrajectory result;
    planner_->Plan(state_, result);

    // if (planner_->Plan(state_, result)) {
    //   double dt = config_.tf / (double) (config_.nfe - 1);
    //   for (int i = 0; i < config_.nfe; i++) {
    //     double time = dt * i;
    //     auto dynamic_obstacles = env_->QueryDynamicObstacles(time);
    //     for (auto &obstacle: dynamic_obstacles) {
    //       int hue = int((double) obstacle.first / env_->dynamic_obstacles().size() * 320);

    //       visualization::PlotPolygon(obstacle.second, 0.2, visualization::Color::fromHSV(hue, 1.0, 1.0), obstacle.first,
    //                                  "Online Obstacle");
    //     }

    //     auto &pt = result.data().at(i);
    //     PlotVehicle(1, {pt.x, pt.y, pt.theta}, atan(pt.kappa * config_.vehicle.wheel_base));
    //     ros::Duration(dt).sleep();
    //   }

    //   visualization::Trigger();
    // }
  }
  Env env_;

private:
  ros::NodeHandle nh_;
  pjpo_curvature::CartesianPlannerConfig config_;
  
  std::shared_ptr<pjpo_curvature::PJPOCurvature> planner_;
  PJPOCurvature::StartState state_;


  ros::Subscriber center_line_subscriber_, obstacles_subscriber_, dynamic_obstacles_subscriber_, goal_subscriber_;

  void PlotVehicle(int id, const math::Pose &pt, double phi) {
    auto tires = GenerateTireBoxes({pt.x(), pt.y(), pt.theta()}, phi);

    int tire_id = 1;
    for (auto &tire: tires) {
      visualization::PlotPolygon(math::Polygon2d(tire), 0.1, visualization::Color::White, id * (tire_id++),
                                 "Tires");
    }
    visualization::PlotPolygon(math::Polygon2d(config_.vehicle.GenerateBox({pt.x(), pt.y(), pt.theta()})), 0.2,
                               visualization::Color::Yellow, id, "Footprint");
    visualization::Trigger();
  }

  std::array<math::Box2d, 4> GenerateTireBoxes(const math::Pose pose, double phi = 0.0) const {
    auto front_pose = pose.extend(config_.vehicle.wheel_base);
    auto track_width = config_.vehicle.width - 0.195;
    double rear_track_width_2 = track_width / 2, front_track_width_2 = track_width / 2;
    double box_length = 0.6345;
    double sin_t = sin(pose.theta());
    double cos_t = cos(pose.theta());
    return {
      math::Box2d({pose.x() - rear_track_width_2 * sin_t, pose.y() + rear_track_width_2 * cos_t}, pose.theta(),
                  box_length, 0.195),
      math::Box2d({pose.x() + rear_track_width_2 * sin_t, pose.y() - rear_track_width_2 * cos_t}, pose.theta(),
                  box_length, 0.195),
      math::Box2d({front_pose.x() - front_track_width_2 * sin_t, front_pose.y() + front_track_width_2 * cos_t},
                  front_pose.theta() + phi, box_length, 0.195),
      math::Box2d({front_pose.x() + front_track_width_2 * sin_t, front_pose.y() - front_track_width_2 * cos_t},
                  front_pose.theta() + phi, box_length, 0.195),
    };
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "pjpo_curvature_node");

  ros::NodeHandle nh;
  visualization::Init(nh, "map", "pjpo_curvature_markers");

  PJPOCurvatureNode node(nh);
  ros::spin();
  return 0;
}