#pragma once

#include "apf_path_planner/base_planner.hpp"
#include "apf_path_planner/vector2d.hpp"
#include <utility>
#include <vector>

namespace apf_path_planner {

class APFPlanner : public BasePlanner {
public:
  APFPlanner();

  std::vector<Vector2D> plan(const Vector2D& goal_pose) override;

  std::vector<std::pair<Vector2D, Vector2D>> getForceMap(
    int height = 10,
    int width = 10
  );

  void setParameters(
    double attraction_gain,
    double repulsion_gain,
    double radius
  );

private:
  static constexpr uint16_t MAX_ITERATION_COUNT = 10000;

  double attraction_gain_;
  double repulsion_gain_;
  double radius_;

  Vector2D calculateAttractionForce(const Vector2D& pose) const;
  Vector2D calculateObstacleRepulsionForce(const Vector2D& pose) const;
  bool isCloseEnough() const;
  bool isObstacle(int i, int j) const;
};

}  // namespace apf_path_planner
