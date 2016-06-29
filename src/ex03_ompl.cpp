#include "herb.hpp"
#include <iostream>
#include <dart/dart.hpp>
#include <aikido/constraint/TSR.hpp>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/util/RNG.hpp>

using aikido::constraint::TSR;
using aikido::rviz::InteractiveMarkerViewer;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::util::RNGWrapper;
using dart::common::make_unique;

static const std::string topicName { "dart_markers" };
static const double planningTimeout { 10. };
static const int maxNumIkTrials { 10 };

int main(int argc, char** argv)
{
  Herb robot;
  auto rightArmSpace = std::make_shared<MetaSkeletonStateSpace>(robot.getRightArm());

  Eigen::VectorXd goalConfiguration(7);
  goalConfiguration << 5.65, -1.76, -0.26, 1.96, -1.15, 0.87,
      -1.43; // relaxed home
  robot.setConfiguration(rightArmSpace, goalConfiguration);
  auto rng = make_unique<RNGWrapper<std::default_random_engine>>(0);
  auto goalRegion = std::make_shared<TSR>(std::move(rng), robot.getRightEndEffector()->getTransform());

  Eigen::VectorXd startConfiguration(7);
  startConfiguration << 3.68, -1.90, 0.00, 2.20, 0.00, 0.00, 0.00; // home
  robot.setConfiguration(rightArmSpace, startConfiguration);

  auto untimedTrajectory = robot.planToTSR(rightArmSpace, 
                                           robot.getRightEndEffector().get(),
                                           goalRegion, planningTimeout);
  if (!untimedTrajectory)
    throw std::runtime_error("Planning failed.");

  auto timedTrajectory =
      robot.retimeTrajectory(rightArmSpace, untimedTrajectory);

  ros::init(argc, argv, "ex03_ompl");

  InteractiveMarkerViewer viewer(topicName);
  viewer.addSkeleton(robot.getSkeleton());
  viewer.setAutoUpdate(true);

  robot.execute(rightArmSpace, timedTrajectory);

  std::cout << "Press <Ctrl> + C to exit." << std::endl;
  ros::spin();
}
