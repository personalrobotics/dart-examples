#include "herb.hpp"
#include <iostream>
#include <dart/dart.hpp>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <aikido/constraint/TSR.hpp>
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

  const Eigen::Vector3d direction(0, 0, -1);
  const auto distance = 0.3;
  const auto endEffector = robot.getRightEndEffector();
  auto untimedTrajectory = robot.planToEndEffectorOffset(
      rightArmSpace, endEffector.get(),
      direction, distance, planningTimeout);
  if (!untimedTrajectory)
    throw std::runtime_error("Failed to find a solution");

  // Debug
  double epsilon = 0.01;
  auto constraint = std::make_shared<aikido::constraint::TSR>();
  constraint->mT0_w = dart::math::computeTransform(direction / direction.norm(),
                                                   endEffector->getTransform().translation(),
                                                   dart::math::AxisType::AXIS_Z);
  constraint->mTw_e = constraint->mT0_w.inverse() * endEffector->getTransform(); 
  constraint->mBw << -epsilon, epsilon, 
      -epsilon, epsilon, std::min(0., distance), std::max(0., distance),
      -epsilon, epsilon, -epsilon, epsilon, -epsilon, epsilon;
  // Debug

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
