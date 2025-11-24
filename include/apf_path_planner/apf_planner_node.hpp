#pragma once

#include "apf_path_planner/apf_planner.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <memory>

namespace apf_path_planner {

class APFPlannerNode : public rclcpp::Node {
public:
  APFPlannerNode();

private:
  void setupPoseSource();
  Vector2D getCurrentPose();
  Vector2D getCurrentPoseFromTF();

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  nav_msgs::msg::Path convertVectorToPath(const std::vector<Vector2D>& vector_path);
  void publishForceMap(
    const std::vector<std::pair<Vector2D, Vector2D>>& force_map,
    const std::string& frame_id
  );

  APFPlanner apf_planner_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscription_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  Vector2D current_pose_from_odom_;
};

}  // namespace apf_path_planner
