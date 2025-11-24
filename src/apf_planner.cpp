#include "apf_path_planner/apf_planner.hpp"
#include <algorithm>

namespace apf_path_planner {

APFPlanner::APFPlanner()
  : attraction_gain_(10.0),
    repulsion_gain_(100.0),
    radius_(10.0) {}

std::vector<Vector2D> APFPlanner::plan(const Vector2D& goal_pose) {
  std::vector<Vector2D> path;
  goal_pose_ = goal_pose;

  path.push_back(current_pose_);
  uint16_t iteration_count = 0;

  while (!isCloseEnough() && iteration_count < MAX_ITERATION_COUNT) {
    Vector2D attraction_force = calculateAttractionForce(current_pose_);
    Vector2D obstacle_force = calculateObstacleRepulsionForce(current_pose_);
    Vector2D total_force = attraction_force.add(obstacle_force);

    current_pose_ = current_pose_.add(
      total_force.getUnitVector().multiply(map_.info.resolution)
    );

    path.push_back(current_pose_);
    iteration_count++;
  }

  return path;
}

bool APFPlanner::isCloseEnough() const {
  return goal_pose_.subtract(current_pose_).getLength() < map_.info.resolution;
}

Vector2D APFPlanner::calculateAttractionForce(const Vector2D& pose) const {
  return goal_pose_.subtract(pose).multiply(attraction_gain_);
}

Vector2D APFPlanner::calculateObstacleRepulsionForce(const Vector2D& pose) const {
  double influence_radius = radius_ * map_.info.resolution;
  Vector2D repulsion_force(0.0, 0.0);

  for (uint32_t i = 0; i < map_.info.height; i++) {
    for (uint32_t j = 0; j < map_.info.width; j++) {
      if (isObstacle(i, j)) {
        Vector2D obstacle_position(
          j * map_.info.resolution,
          i * map_.info.resolution
        );
        Vector2D diff_vector = pose.subtract(obstacle_position);
        double distance = diff_vector.getLength();

        if (distance <= influence_radius && distance > 1e-10) {
          double force_magnitude = repulsion_gain_ *
            (1.0 / distance - 1.0 / influence_radius) /
            (distance * distance);

          repulsion_force = repulsion_force.add(
            diff_vector.getUnitVector().multiply(force_magnitude)
          );
        }
      }
    }
  }

  return repulsion_force;
}

bool APFPlanner::isObstacle(int i, int j) const {
  uint32_t index = i * map_.info.width + j;
  if (index >= map_.data.size()) {
    return false;
  }
  return map_.data[index] > 50;
}

std::vector<std::pair<Vector2D, Vector2D>> APFPlanner::getForceMap(
  int height,
  int width
) {
  std::vector<std::pair<Vector2D, Vector2D>> force_map;

  for (double i = current_pose_.x - height / 2.0;
       i < current_pose_.x + height / 2.0;
       i += 0.5) {
    for (double j = current_pose_.y - width / 2.0;
         j < current_pose_.y + width / 2.0;
         j += 0.5) {
      Vector2D pose(i, j);
      Vector2D attraction_force = calculateAttractionForce(pose);
      Vector2D obstacle_force = calculateObstacleRepulsionForce(pose);
      Vector2D total_force = attraction_force.add(obstacle_force);

      force_map.push_back({pose, total_force});
    }
  }

  return force_map;
}

void APFPlanner::setParameters(
  double attraction_gain,
  double repulsion_gain,
  double radius
) {
  attraction_gain_ = attraction_gain;
  repulsion_gain_ = repulsion_gain;
  radius_ = radius;
}

}  // namespace apf_path_planner
