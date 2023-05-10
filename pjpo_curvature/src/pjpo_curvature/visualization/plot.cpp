#include "pjpo_curvature/visualization/plot.h"
#include "pjpo_curvature/CenterLine.h"



namespace pjpo_curvature {
namespace visualization {
namespace {
std::string frame_ = "map";
std::mutex mutex_;

ros::Publisher publisher_;
ros::Publisher kappa_publisher_;
ros::Publisher l_publisher_;
ros::Publisher dl_publisher_;
ros::Publisher ddl_publisher_;
ros::Publisher dddl_publisher_;
visualization_msgs::MarkerArray arr_;

ros::Publisher sl_path_publisher_;


}

void Init(ros::NodeHandle &node, const std::string &frame, const std::string &topic) {
  frame_ = frame;
  publisher_ = node.advertise<visualization_msgs::MarkerArray>(topic, 10, true);
  kappa_publisher_ = node.advertise<geometry_msgs::PoseArray>("kappa_variables", 1, true);
  l_publisher_ = node.advertise<geometry_msgs::PoseArray>("l_variables", 1, true);
  dl_publisher_ = node.advertise<geometry_msgs::PoseArray>("dl_variables", 1, true);
  ddl_publisher_ = node.advertise<geometry_msgs::PoseArray>("ddl_variables", 1, true);
  dddl_publisher_ = node.advertise<geometry_msgs::PoseArray>("dddl_variables", 1, true);
  sl_path_publisher_ = node.advertise<pjpo_curvature::FrenetPath>("sl_paths", 10, true);
  
}

void
Plot(const Vector &xs, const Vector &ys, double width, Color color, int id, const std::string &ns) {
  visualization_msgs::Marker msg;
  msg.header.frame_id = frame_;
  msg.header.stamp = ros::Time();
  msg.ns = ns;
  msg.id = id >= 0 ? id : arr_.markers.size();

  msg.action = visualization_msgs::Marker::ADD;
  msg.type = visualization_msgs::Marker::LINE_STRIP;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = width;
  msg.color = color.toColorRGBA();

  for (size_t i = 0; i < xs.size(); i++) {
    geometry_msgs::Point pt;
    pt.x = xs[i];
    pt.y = ys[i];
    pt.z = 0.1 * id;
    msg.points.push_back(pt);
  }

  mutex_.lock();
  arr_.markers.push_back(msg);
  mutex_.unlock();
}

void Plot(const Vector &xs, const Vector &ys, double width,
          const std::vector<Color> &color, int id, const std::string &ns) {
  assert(xs.size() == color.size());

  visualization_msgs::Marker msg;
  msg.header.frame_id = frame_;
  msg.header.stamp = ros::Time();
  msg.ns = ns;
  msg.id = id >= 0 ? id : arr_.markers.size();

  msg.action = visualization_msgs::Marker::ADD;
  msg.type = visualization_msgs::Marker::LINE_STRIP;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = width;

  for (size_t i = 0; i < xs.size(); i++) {
    geometry_msgs::Point pt;
    pt.x = xs[i];
    pt.y = ys[i];
    msg.points.push_back(pt);
    msg.colors.push_back(color[i].toColorRGBA());
  }

  mutex_.lock();
  arr_.markers.push_back(msg);
  mutex_.unlock();
}


void PlotPolygon(const Vector &xs, const Vector &ys, double width, Color color, int id,
                 const std::string &ns) {
  auto xxs = xs;
  auto yys = ys;
  xxs.push_back(xxs[0]);
  yys.push_back(yys[0]);
  Plot(xxs, yys, width, color, id, ns);
}

void PlotPolygon(const Polygon2d &polygon, double width, Color color, int id,
                 const std::string &ns) {
  std::vector<double> xs, ys;
  for (auto &pt: polygon.points()) {
    xs.push_back(pt.x());
    ys.push_back(pt.y());
  }
  PlotPolygon(xs, ys, width, color, id, ns);
}

void PlotTrajectory(const Vector &xs, const Vector &ys, const Vector &vs, double max_velocity, double width,
                    const Color &color, int id, const std::string &ns) {
  std::vector<Color> colors(xs.size());
  float h, tmp;
  color.toHSV(h, tmp, tmp);

  for (size_t i = 0; i < xs.size(); i++) {
    double percent = (vs[i] / max_velocity);
    colors[i] = Color::fromHSV(h, percent, 1.0);
  }

  Plot(xs, ys, width, colors, id, ns);
}

void PlotPoints(const Vector &xs, const Vector &ys, double width, const Color &color, int id,
                const std::string &ns) {
  assert(xs.size() == ys.size());

  visualization_msgs::Marker msg;
  msg.header.frame_id = frame_;
  msg.header.stamp = ros::Time();
  msg.ns = ns.empty() ? "Points" : ns;
  msg.id = id >= 0 ? id : arr_.markers.size();

  msg.action = visualization_msgs::Marker::ADD;
  msg.type = visualization_msgs::Marker::POINTS;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = msg.scale.y = width;
  msg.color = color.toColorRGBA();

  for (size_t i = 0; i < xs.size(); i++) {
    geometry_msgs::Point pt;
    pt.x = xs[i];
    pt.y = ys[i];
    msg.points.push_back(pt);
  }

  mutex_.lock();
  arr_.markers.push_back(msg);
  mutex_.unlock();
}

void Trigger() {
  mutex_.lock();
  publisher_.publish(arr_);
  arr_.markers.clear();
  mutex_.unlock();
}

void SLdraw(const geometry_msgs::PoseArray & l_,
            const geometry_msgs::PoseArray & dl_,
            const geometry_msgs::PoseArray & ddl_,
            const geometry_msgs::PoseArray & dddl_){
  l_publisher_.publish(l_);
  dl_publisher_.publish(dl_);
  ddl_publisher_.publish(ddl_);
  dddl_publisher_.publish(dddl_);
}

void Clear() {
  mutex_.lock();
  arr_.markers.clear();

  visualization_msgs::MarkerArray arr;
  visualization_msgs::Marker msg;
  msg.header.frame_id = frame_;
  msg.ns = "Markers";

  msg.action = visualization_msgs::Marker::DELETEALL;
  arr.markers.push_back(msg);
  publisher_.publish(arr);
  mutex_.unlock();
}

void PubKappaInfo(const geometry_msgs::PoseArray & kappa_info){
  kappa_publisher_.publish(kappa_info);
}
  
void PubSLPath(const pjpo_curvature::FrenetPath & path){
  sl_path_publisher_.publish(path);
}

}
}