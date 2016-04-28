#include "herb.hpp"
#include <iostream>
#include <dart/dart.h>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>

using aikido::rviz::InteractiveMarkerViewer;
using aikido::statespace::dart::MetaSkeletonStateSpace;

static const std::string topicName{"dart_markers"};
static const double planningTimeout{60.};

int main(int argc, char **argv) {
  Herb robot;

  auto rightArmSpace =
      std::make_shared<MetaSkeletonStateSpace>(robot.getRightArm());

  Eigen::VectorXd startConfiguration(7);
  startConfiguration << 3.68, -1.90, 0.00, 2.20, 0.00, 0.00, 0.00; // home
  robot.setConfiguration(rightArmSpace, startConfiguration);

  auto untimedTrajectory = robot.planToEndEffectorOffset(
      rightArmSpace, robot.getRightEndEffector().get(),
      Eigen::Vector3d(0, 0, -1),
      0.3, planningTimeout);
  if (!untimedTrajectory)
    throw std::runtime_error("Failed to find a solution");

  auto timedTrajectory =
      robot.retimeTrajectory(rightArmSpace, untimedTrajectory);

  ros::init(argc, argv, "ex04_ompl");

  InteractiveMarkerViewer viewer(topicName);
  viewer.addSkeleton(robot.getSkeleton());
  viewer.setAutoUpdate(true);

  robot.execute(rightArmSpace, timedTrajectory);

  std::cout << "Press <Ctrl> + C to exit." << std::endl;
  ros::spin();
}
