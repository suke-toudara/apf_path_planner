#include "apf_path_planner/apf_planner_node.hpp"
#include <tf2/LinearMath/Quaternion.h>

namespace apf_path_planner {

APFPlannerNode::APFPlannerNode() : Node("apf_planner_node") {
  this->declare_parameter("attraction_gain", 10.0);
  this->declare_parameter("repulsion_gain", 100.0);
  this->declare_parameter("radius", 10.0);
  this->declare_parameter("forcemap_width", 10);
  this->declare_parameter("forcemap_height", 10);
  this->declare_parameter("pose_source", "tf");
  this->declare_parameter("robot_frame_id", "base_link");
  this->declare_parameter("map_frame_id", "map");
  this->declare_parameter("odom_topic", "/odom");

  double attraction_gain = this->get_parameter("attraction_gain").as_double();
  double repulsion_gain = this->get_parameter("repulsion_gain").as_double();
  double radius = this->get_parameter("radius").as_double();
  apf_planner_.setParameters(attraction_gain, repulsion_gain, radius);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/force_arrows", 10
  );

  map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10,
    std::bind(&APFPlannerNode::mapCallback, this, std::placeholders::_1)
  );

  goal_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 10,
    std::bind(&APFPlannerNode::goalPoseCallback, this, std::placeholders::_1)
  );

  setupPoseSource();
}

void APFPlannerNode::setupPoseSource() {
  std::string pose_source = this->get_parameter("pose_source").as_string();

  if (pose_source == "odom") {
    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      std::bind(&APFPlannerNode::odomCallback, this, std::placeholders::_1)
    );
  }
}

Vector2D APFPlannerNode::getCurrentPose() {
  std::string pose_source = this->get_parameter("pose_source").as_string();

  if (pose_source == "tf") {
    return getCurrentPoseFromTF();
  } else if (pose_source == "odom") {
    return current_pose_from_odom_;
  }

  return Vector2D(0.0, 0.0);
}

Vector2D APFPlannerNode::getCurrentPoseFromTF() {
  std::string robot_frame = this->get_parameter("robot_frame_id").as_string();
  std::string map_frame = this->get_parameter("map_frame_id").as_string();

  try {
    auto transform = tf_buffer_->lookupTransform(
      map_frame, robot_frame, tf2::TimePointZero
    );

    return Vector2D(
      transform.transform.translation.x,
      transform.transform.translation.y
    );
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
    return Vector2D(0.0, 0.0);
  }
}

void APFPlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  apf_planner_.setMap(*msg);
}

void APFPlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_pose_from_odom_ = Vector2D(
    msg->pose.pose.position.x,
    msg->pose.pose.position.y
  );
}

void APFPlannerNode::goalPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg
) {
  Vector2D goal_position(msg->pose.position.x, msg->pose.position.y);

  Vector2D current_pose = getCurrentPose();
  apf_planner_.setCurrentPose(current_pose);

  std::vector<Vector2D> path_vector = apf_planner_.plan(goal_position);

  nav_msgs::msg::Path path_msg = convertVectorToPath(path_vector);
  path_msg.header.frame_id = msg->header.frame_id;
  path_msg.header.stamp = this->now();
  path_publisher_->publish(path_msg);

  int forcemap_width = this->get_parameter("forcemap_width").as_int();
  int forcemap_height = this->get_parameter("forcemap_height").as_int();

  std::vector<std::pair<Vector2D, Vector2D>> force_map =
    apf_planner_.getForceMap(forcemap_height, forcemap_width);

  publishForceMap(force_map, msg->header.frame_id);
}

nav_msgs::msg::Path APFPlannerNode::convertVectorToPath(
  const std::vector<Vector2D>& vector_path
) {
  nav_msgs::msg::Path path;

  for (const auto& point : vector_path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  return path;
}

void APFPlannerNode::publishForceMap(
  const std::vector<std::pair<Vector2D, Vector2D>>& force_map,
  const std::string& frame_id
) {
  visualization_msgs::msg::MarkerArray marker_array;
  int marker_id = 0;

  for (const auto& [pose, force] : force_map) {
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = frame_id;
    arrow.header.stamp = this->now();
    arrow.id = marker_id++;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;

    arrow.pose.position.x = pose.x;
    arrow.pose.position.y = pose.y;
    arrow.pose.position.z = 1.0;

    Vector2D unit_force = force.getUnitVector();
    double yaw = std::atan2(unit_force.y, unit_force.x);

    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw);

    arrow.pose.orientation.x = quaternion.x();
    arrow.pose.orientation.y = quaternion.y();
    arrow.pose.orientation.z = quaternion.z();
    arrow.pose.orientation.w = quaternion.w();

    arrow.scale.x = 0.5;
    arrow.scale.y = 0.05;
    arrow.scale.z = 0.05;

    arrow.color.r = 0.0;
    arrow.color.g = 1.0;
    arrow.color.b = 0.0;
    arrow.color.a = 1.0;

    marker_array.markers.push_back(arrow);
  }

  marker_publisher_->publish(marker_array);
}

}  // namespace apf_path_planner

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<apf_path_planner::APFPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
