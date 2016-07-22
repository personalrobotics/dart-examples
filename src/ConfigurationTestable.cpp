#include "ConfigurationTestable.hpp"
#include <Eigen/Dense>

using namespace aikido;
using statespace::dart::MetaSkeletonStateSpace;

ConfigurationTestable::ConfigurationTestable(
    statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
    const statespace::StateSpace::State* goalState)
    : mStateSpace{std::move(stateSpace)}
// , goalState{std::move(goalState)}
{
  if (!mStateSpace) throw std::invalid_argument("stateSpace is nullptr");
  if (!goalState) throw std::invalid_argument("goalState is nullptr");

  auto cartGoalState =
      static_cast<const MetaSkeletonStateSpace::State*>(goalState);
  mStateSpace->convertStateToPositions(cartGoalState, mGoalConfiguration);
}

// Documentation inherited
bool ConfigurationTestable::isSatisfied(
    const statespace::StateSpace::State* _state) const
{
  auto current_state =
      static_cast<const MetaSkeletonStateSpace::State*>(_state);
  Eigen::VectorXd curr_position;
  mStateSpace->convertStateToPositions(current_state, curr_position);
  return curr_position.isApprox(mGoalConfiguration);
}

// Documentation inherited
std::shared_ptr<statespace::StateSpace> ConfigurationTestable::getStateSpace()
    const
{
  return mStateSpace;
}
