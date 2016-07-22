#include "herb.hpp"
#include "DistanceConstraint.hpp"
#include <iostream>
#include <dart/dart.hpp>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <aikido/constraint/FrameDifferentiable.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/planner/mintos/MinTOS.hpp>

using aikido::rviz::InteractiveMarkerViewer;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::planner::mintos::interpolateAndTimeOptimizeTrajectory;
using aikido::trajectory::Spline;

static const std::string topicName{"dart_markers"};
static const double planningTimeout{60.};

int main(int argc, char **argv)
{
  Herb robot;

  auto rightArm = robot.getRightArm();
  auto rightArmSpace = std::make_shared<MetaSkeletonStateSpace>(rightArm);
  Eigen::VectorXd startConfiguration(7);
  // startConfiguration << 3.68, -1.90, 0.00, 2.20, 0.00, 0.00, 0.00; // home
  startConfiguration << 1.55, -0.83, 0.0, 0.0, 0.0, 0.85,
      0.0;  // straight swept left
  robot.setConfiguration(rightArmSpace, startConfiguration);

  auto leftArm = robot.getLeftArm();
  auto leftArmSpace = std::make_shared<MetaSkeletonStateSpace>(leftArm);
  startConfiguration << 0.64, -1.76, 0.26, 1.96, 1.16, 0.87,
      1.43;  // relaxed_home
  robot.setConfiguration(leftArmSpace, startConfiguration);

  Eigen::VectorXd endConfiguration{7};
  endConfiguration << 1.55, 0.83, 0.0, 0.0, 0.0, -0.39, 0.0;

  auto endEffector = robot.getRightEndEffector();
  auto untimedTrajectory = robot.planToConfiguration(
      rightArmSpace, endEffector.get(), endConfiguration, planningTimeout);
  if (!untimedTrajectory) throw std::runtime_error("Failed to find a solution");

  Eigen::Vector3d point_to_avoid{0.88, -0.12, 0.91};  // in front of Herb
  auto constraint = std::make_shared<DistanceConstraint>(point_to_avoid, 0.2);

  aikido::constraint::FrameDifferentiable actualConstraint(
      rightArmSpace, endEffector.get(), constraint);

  const auto velocityLimits = robot.getVelocityLimits(*rightArm);
  const auto accelerationLimits = robot.getAccelerationLimits(*rightArm);
  const auto constraintTolerance = 0.05;
  const auto interpolationTimestep = 0.05;

  std::cout << "Retiming with MinTOS..." << std::endl;
  std::shared_ptr<Spline> mintosTrajectory =
      interpolateAndTimeOptimizeTrajectory(
          *untimedTrajectory, actualConstraint, -velocityLimits, velocityLimits,
          -accelerationLimits, accelerationLimits, constraintTolerance,
          interpolationTimestep);

  std::cout << "Retiming with parabolic..." << std::endl;
  auto parabolicTrajectory =
      robot.retimeTrajectory(rightArmSpace, untimedTrajectory);

  std::cout << "Initializing ROS and viewer..." << std::endl;
  ros::init(argc, argv, "ex05_mintos");

  InteractiveMarkerViewer viewer(topicName);
  viewer.addSkeleton(robot.getSkeleton());
  viewer.setAutoUpdate(true);

  std::cout << "Press Return to continue..." << std::endl;
  std::cin.get();
  ros::WallDuration(5).sleep();

  std::cout << "Executing MinTOS trajectory..." << std::endl;
  robot.execute(rightArmSpace, mintosTrajectory);

  ros::WallDuration(2).sleep();
  std::cout << "Executing parabolic trajectory..." << std::endl;
  robot.execute(rightArmSpace, parabolicTrajectory);

  std::cout << "Press <Ctrl> + C to exit." << std::endl;
  ros::spin();
}
