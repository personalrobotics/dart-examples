#include "DistanceConstraint.hpp"
#include <cassert>
#include <cmath>
#include <dart/math/math.hpp>

DistanceConstraint::DistanceConstraint(const Eigen::Vector3d& point,
                                       const double distance)
    : point{point}
    , distance{distance}
    , stateSpace{std::make_shared<statespace::SE3>()} {};

DistanceConstraint::DistanceConstraint(const DistanceConstraint& other)
    : point{other.point}
    , distance{other.distance}
    , stateSpace{std::make_shared<statespace::SE3>()} {};

DistanceConstraint::DistanceConstraint(DistanceConstraint&& other)
    : point{other.point}
    , distance{other.distance}
    , stateSpace{std::make_shared<statespace::SE3>()} {};

DistanceConstraint& DistanceConstraint::operator=(
    const DistanceConstraint& other)
{
  point = other.point;
  distance = other.distance;
  stateSpace = std::make_shared<statespace::SE3>();

  return *this;
}

DistanceConstraint& DistanceConstraint::operator=(DistanceConstraint&& other)
{
  point = std::move(other.point);
  distance = other.distance;
  stateSpace = std::move(other.stateSpace);

  return *this;
}

statespace::StateSpacePtr DistanceConstraint::getStateSpace() const
{
  return stateSpace;
}

std::shared_ptr<statespace::SE3> DistanceConstraint::getSE3() const
{
  return stateSpace;
}

std::vector<constraint::ConstraintType> DistanceConstraint::getConstraintTypes()
    const
{
  return std::vector<constraint::ConstraintType>(
      1, constraint::ConstraintType::INEQUALITY);
}

bool DistanceConstraint::isSatisfied(
    const statespace::StateSpace::State* _s) const
{
  static constexpr double eps = 1e-6;
  Eigen::VectorXd dist;
  getValue(_s, dist);
  return dist.norm() < eps;
};

size_t DistanceConstraint::getConstraintDimension() const { return 1; }

void DistanceConstraint::getValue(const statespace::StateSpace::State* _s,
                                  Eigen::VectorXd& _out) const
{
  auto se3state = static_cast<const statespace::SE3::State*>(_s);

  Eigen::Vector3d dist3d = se3state->getIsometry().translation() - point;

  _out.resize(1);

  _out[0] = abs(dist3d.norm() - distance);
}

void DistanceConstraint::getJacobian(const statespace::StateSpace::State* _s,
                                     Eigen::MatrixXd& _out) const
{
  _out.resize(1, 6);

  auto se3state = static_cast<const statespace::SE3::State*>(_s);
  Eigen::Isometry3d se3 = se3state->getIsometry();

  Eigen::Vector3d state_pt = se3state->getIsometry().translation();
  Eigen::Vector3d dist3d = state_pt - point;
  double d2 = dist3d.squaredNorm();

  double x = state_pt[0];
  double x_0 = point[0];
  double y = state_pt[1];
  double y_0 = point[1];
  double z = state_pt[2];
  double z_0 = point[2];

  assert(pow(x - x_0, 2) + pow(y - y_0, 2) + pow(z - z_0, 2) - d2 < 1e-4);

  // clang-format off
  _out(0,0) = 2*x - x_0 + pow(y, 2) - y*y_0 + pow(y_0, 2) + pow(z, 2) - z*z_0 + pow(z_0, 2);  // ∂d2/∂x
  _out(0,1) = pow(x, 2) - x*x_0 + pow(x_0, 2) + 2*y - y_0 + pow(z, 2) - z*z_0 + pow(z_0, 2);  // ∂d2/∂y
  _out(0,2) = pow(x, 2) - x*x_0 + pow(x_0, 2) + pow(y, 2) - y*y_0 + pow(y_0, 2) + 2*z - z_0;  // ∂d2/∂z
  // clang-format on

  // Eigen::Vector3d gradient = dist3d / dist3d.norm();

  for (int i = 3; i < 6; ++i) {
    _out(0, i) = 0.0;
  }
}
