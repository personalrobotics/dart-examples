#ifndef DARTEXAMPLES_CONFIGURATIONTESTABLE_HPP_
#define DARTEXAMPLES_CONFIGURATIONTESTABLE_HPP_

#include <memory>
#include <aikido/constraint/Testable.hpp>
#include <aikido/statespace/StateSpace.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>

using namespace aikido;

class ConfigurationTestable : public constraint::Testable
{
public:
  ConfigurationTestable(statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
                        const statespace::StateSpace::State goalState);

  // Documentation inherited
  bool isSatisfied(const StateSpace::State* _state) const override;

  // Documentation inherited
  std::shared_ptr<statespace::StateSpace> getStateSpace() const override;

private:
  statespace::dart::MetaSkeletonStateSpacePtr stateSpace;
  // std::shared_ptr<statespace::SE3> goalStateSpace;
  // statespace::StateSpace::State goalState;
  Eigen::VectorXd goalConfiguration;
};

#endif  // DARTEXAMPLES_CONFIGURATIONTESTABLE_HPP_
