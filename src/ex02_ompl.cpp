#include "herb.hpp"
#include <iostream>
#include <dart/dart.h>
#include <aikido/constraint/FrameTestable.hpp>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>

using aikido::rviz::InteractiveMarkerViewer;
using aikido::statespace::dart::MetaSkeletonStateSpace;

static const std::string topicName{"dart_markers"};
static const double planningTimeout{10.};

int main(int argc, char **argv) {
  Herb robot;

  auto rightArmSpace =
      std::make_shared<MetaSkeletonStateSpace>(robot.getRightArm());

  Eigen::VectorXd startConfiguration(7);
  startConfiguration << 3.68, -1.90, 0.00, 2.20, 0.00, 0.00, 0.00; // home
  robot.setConfiguration(rightArmSpace, startConfiguration);

  Eigen::VectorXd goalConfiguration(7);
  goalConfiguration << 5.65, -1.76, -0.26, 1.96, -1.15, 0.87,
      -1.43; // relaxed home

  auto untimedTrajectory = robot.planToConfiguration(
      rightArmSpace, goalConfiguration, planningTimeout);
  if (!untimedTrajectory)
    throw std::runtime_error("Failed to find a solution");

  auto timedTrajectory =
      robot.retimeTrajectory(rightArmSpace, untimedTrajectory);

  ros::init(argc, argv, "ex02_ompl");

  InteractiveMarkerViewer viewer(topicName);
  viewer.addSkeleton(robot.getSkeleton());
  viewer.setAutoUpdate(true);

  robot.execute(rightArmSpace, timedTrajectory);

  std::cout << "Press <Ctrl> + C to exit." << std::endl;
  ros::spin();
}
