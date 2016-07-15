#include "herb.hpp"
#include <iostream>
#include <dart/dart.hpp>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/planner/mintos/MinTOS.hpp>

using aikido::rviz::InteractiveMarkerViewer;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::planner::mintos::interpolateAndTimeOptimizeTrajectory;
using aikido::trajectory::Spline;

static const std::string topicName{"dart_markers"};
static const double planningTimeout{60.};

int main(int argc, char **argv) {
  Herb robot;

  auto rightArm = robot.getRightArm();
  auto rightArmSpace = std::make_shared<MetaSkeletonStateSpace>(rightArm);

  Eigen::VectorXd startConfiguration(7);
  startConfiguration << 3.68, -1.90, 0.00, 2.20, 0.00, 0.00, 0.00; // home
  robot.setConfiguration(rightArmSpace, startConfiguration);

  auto untimedTrajectory = robot.planToEndEffectorOffset(
      rightArmSpace, robot.getRightEndEffector().get(),
      Eigen::Vector3d(0, 0, -1),
      0.3, planningTimeout);
  if (!untimedTrajectory)
    throw std::runtime_error("Failed to find a solution");

  aikido::constraint::Differentiable* constraint = nullptr;
  const auto velocityLimits = robot.getVelocityLimits(*rightArm);
  const auto accelerationLimits = robot.getAccelerationLimits(*rightArm);
  const auto constraintTolerance = 0.01;
  const auto interpolationTimestep = 0.05;

  std::shared_ptr<Spline> timedTrajectory
    = interpolateAndTimeOptimizeTrajectory(
        *untimedTrajectory, *constraint,
        -velocityLimits, velocityLimits,
        -accelerationLimits, accelerationLimits,
        constraintTolerance, interpolationTimestep);

#if 0
  ros::init(argc, argv, "ex05_mintos");

  InteractiveMarkerViewer viewer(topicName);
  viewer.addSkeleton(robot.getSkeleton());
  viewer.setAutoUpdate(true);

  robot.execute(rightArmSpace, timedTrajectory);

  std::cout << "Press <Ctrl> + C to exit." << std::endl;
  ros::spin();
#endif
}
