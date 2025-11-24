#pragma once

#include <cmath>

namespace apf_path_planner {

class Vector2D {
public:
  double x;
  double y;

  Vector2D() : x(0.0), y(0.0) {}

  Vector2D(double x_val, double y_val) : x(x_val), y(y_val) {}

  Vector2D add(const Vector2D& other) const {
    return Vector2D(x + other.x, y + other.y);
  }

  Vector2D subtract(const Vector2D& other) const {
    return Vector2D(x - other.x, y - other.y);
  }

  Vector2D multiply(double scalar) const {
    return Vector2D(x * scalar, y * scalar);
  }

  double getLength() const {
    return std::sqrt(x * x + y * y);
  }

  Vector2D getUnitVector() const {
    double length = getLength();
    if (length < 1e-10) {
      return Vector2D(0.0, 0.0);
    }
    return Vector2D(x / length, y / length);
  }

  Vector2D operator+(const Vector2D& other) const {
    return add(other);
  }

  Vector2D operator-(const Vector2D& other) const {
    return subtract(other);
  }

  Vector2D operator*(double scalar) const {
    return multiply(scalar);
  }
};

}  // namespace apf_path_planner
