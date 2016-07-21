#ifndef DARTEXAMPLES_DISTANCECONSTRAINT_HPP_
#define DARTEXAMPLES_DISTANCECONSTRAINT_HPP_

#include <memory>
#include <aikido/constraint/Differentiable.hpp>
#include <aikido/statespace/StateSpace.hpp>
#include <aikido/constraint/Testable.hpp>
#include <aikido/statespace/SE3.hpp>
#include <Eigen/Dense>

using namespace aikido;

class DistanceConstraint : public aikido::constraint::Differentiable,
                           public aikido::constraint::Testable
{
public:
  // enum ConstraintType { MIN_DISTANCE, MAX_DISTANCE };

  DistanceConstraint(const Eigen::Vector3d& point = Eigen::Vector3d::Zero(),
                     const double distance = 0);
  // const ConstraintType type = DistanceConstraint::MIN_DISTANCE);

  DistanceConstraint(const DistanceConstraint& other);
  DistanceConstraint(DistanceConstraint&& other);

  DistanceConstraint& operator=(const DistanceConstraint& other);
  DistanceConstraint& operator=(DistanceConstraint&& other);

  virtual ~DistanceConstraint() = default;

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited
  std::shared_ptr<statespace::SE3> getSE3() const;

  // Documentation inherited
  std::vector<constraint::ConstraintType> getConstraintTypes() const override;

  // Documentation inherited.
  bool isSatisfied(const statespace::StateSpace::State* _s) const override;

  // Documentation inherited.
  size_t getConstraintDimension() const override;

  // Documentation inherited.
  void getValue(const statespace::StateSpace::State* _s,
                Eigen::VectorXd& _out) const override;

  // Documentation inherited
  void getJacobian(const statespace::StateSpace::State* _s,
                   Eigen::MatrixXd& _out) const override;

  Eigen::Vector3d point;
  double distance;
  // ConstraintType type;

private:
  std::shared_ptr<statespace::SE3> stateSpace;
};

#endif  // DARTEXAMPLES_DISTANCECONSTRAINT_HPP_
