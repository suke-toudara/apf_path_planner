#pragma once

#include "apf_path_planner/vector2d.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>

namespace apf_path_planner {

class BasePlanner {
public:
  virtual ~BasePlanner() = default;

  virtual std::vector<Vector2D> plan(const Vector2D& goal_pose) = 0;

  virtual void setCurrentPose(const Vector2D& start_pose) {
    current_pose_ = start_pose;
  }

  virtual void setMap(const nav_msgs::msg::OccupancyGrid& map) {
    map_ = map;
  }

protected:
  nav_msgs::msg::OccupancyGrid map_;
  Vector2D goal_pose_;
  Vector2D current_pose_;
};

}  // namespace apf_path_planner
