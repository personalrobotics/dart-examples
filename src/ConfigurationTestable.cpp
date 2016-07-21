#include "ConfigurationTestable.hpp"
#include <Eigen/Dense>

using namespace aikido;
using statespace::dart::MetaSkeletonStateSpace;

ConfigurationTestable::ConfigurationTestable(
    statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
    const statespace::StateSpace::State goalState)
    : stateSpace{std::move(stateSpace)}
    , goalConfiguration{}
// , goalState{std::move(goalState)}
{
  if (!stateSpace) throw std::invalid_argument("stateSpace is nullptr");
  if (!goalState) throw std::invalid_argument("goalState is null");
  // goalStateSpace = std::dynamic_cast<statespace::SE3>(goalState);
  // if (!goalStateSpace) throw std::invalid_argument("goalState is not in
  // SE3");

  stateSpace->convertStateToPositions(goalState, goalConfiguration);
}

// Documentation inherited
bool ConfigurationTestable::isSatisfied(const StateSpace::State* _state) const
{
  // auto currentState = static_cast<const
  // MetaSkeletonStateSpace::State*>(_state);
  Eigen::VectorXd curr_position;
  stateSpace->convertStateToPositions(_state, curr_position);
  return curr_position.isApprox(goalConfiguration);
}

// Documentation inherited
std::shared_ptr<statespace::StateSpace> ConfigurationTestable::getStateSpace()
    const
{
  return stateSpace;
}
