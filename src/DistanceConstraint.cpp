#include "DistanceConstraint.hpp"
#include <cmath>
#include <dart/math/math.hpp>

DistanceConstraint::DistanceConstraint(const Eigen::Vector3d& center,
                                       const double radius)
    : center{center}
    , radius{radius}
    , stateSpace{std::make_shared<statespace::SE3>()} {};

DistanceConstraint::DistanceConstraint(const DistanceConstraint& other)
    : center{other.center}
    , radius{other.radius}
    , stateSpace{std::make_shared<statespace::SE3>()} {};

DistanceConstraint::DistanceConstraint(DistanceConstraint&& other)
    : center{other.center}
    , radius{other.radius}
    , stateSpace{std::make_shared<statespace::SE3>()} {};

DistanceConstraint& DistanceConstraint::operator=(
    const DistanceConstraint& other)
{
  center = other.center;
  radius = other.radius;
  stateSpace = std::make_shared<statespace::SE3>();

  return *this;
}

DistanceConstraint& DistanceConstraint::operator=(DistanceConstraint&& other)
{
  center = std::move(other.center);
  radius = other.radius;
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
  return std::vector<constraint::ConstraintType>{
    constraint::ConstraintType::EQUALITY};
}

bool DistanceConstraint::isSatisfied(
    const statespace::StateSpace::State* _s) const
{
  static constexpr double eps = 1e-6;
  Eigen::VectorXd dist;
  getValue(_s, dist);
  return dist.norm() < eps;
};

size_t DistanceConstraint::getConstraintDimension() const
{
  return 1;
}

void DistanceConstraint::getValue(const statespace::StateSpace::State* _s,
                                  Eigen::VectorXd& _out) const
{
  auto se3state = static_cast<const statespace::SE3::State*>(_s);
  const auto pose = se3state->getIsometry();
  const auto displacement = (center - pose.translation()).eval();

  _out.resize(1);
  _out[0] = std::pow(radius, 2) - displacement.squaredNorm();
}

void DistanceConstraint::getJacobian(const statespace::StateSpace::State* _s,
                                     Eigen::MatrixXd& _out) const
{
  auto se3state = static_cast<const statespace::SE3::State*>(_s);
  const auto pose = se3state->getIsometry();
  const auto displacement = (center - pose.translation()).eval();

  _out.resize(1, 6);
  _out.leftCols<3>().setZero();
  _out.rightCols<3>() = 2. * displacement.transpose();
}
