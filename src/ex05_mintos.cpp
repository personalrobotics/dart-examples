#include "herb.hpp"
#include "DistanceConstraint.hpp"
#include <iostream>
#include <dart/dart.hpp>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <aikido/constraint/FrameDifferentiable.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/planner/mintos/MinTOS.hpp>
#include <aikido/rviz/ShapeFrameMarker.hpp>

using aikido::rviz::InteractiveMarkerViewer;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::planner::mintos::interpolateAndTimeOptimizeTrajectory;
using aikido::trajectory::Spline;
using dart::dynamics::Frame;
using dart::dynamics::SimpleFrame;

static const std::string topicName{"dart_markers"};
static const double planningTimeout{60.};
static const double constraintRadius{0.2};

int main(int argc, char **argv)
{
  Herb robot;

  auto rightArm = robot.getRightArm();
  auto rightArmSpace = std::make_shared<MetaSkeletonStateSpace>(rightArm);
  auto endEffector = robot.getRightEndEffector();
  auto endEffectorIk = endEffector->getOrCreateIK();

  const auto endEffectorFrame = std::make_shared<SimpleFrame>(
    endEffector, "ee");

  // Hard-coded start configuration.
  Eigen::VectorXd rightStartConfiguration(7);
  rightStartConfiguration
    << 5.65, -1.09011077, -0.26,  1.96, -1.15, 0.81463122, -2.37836581;
  robot.setConfiguration(rightArmSpace, rightStartConfiguration);
  const auto startPose = endEffector->getTransform();
  const auto startFrame = std::make_shared<SimpleFrame>(
    Frame::World(), "start", startPose);

  // Compute a goal configuration.
  Eigen::Isometry3d goalPose = endEffector->getTransform();
  goalPose.pretranslate(Eigen::Vector3d(0., -2. * constraintRadius, 0.));
  const auto goalFrame = std::make_shared<SimpleFrame>(
    Frame::World(), "goal", goalPose);
  endEffectorIk->setTarget(goalFrame);

  Eigen::VectorXd goalConfiguration;
  if (!endEffectorIk->solve(goalConfiguration, false))
    throw std::runtime_error("Failed to find a goal IK solution.");

  // Set the left arm into a safe configuration.
  auto leftArm = robot.getLeftArm();
  auto leftArmSpace = std::make_shared<MetaSkeletonStateSpace>(leftArm);
  Eigen::VectorXd leftStartConfiguration(7);
  leftStartConfiguration << 0.64, -1.76, 0.26, 1.96, 1.16, 0.87, 1.43;
  robot.setConfiguration(leftArmSpace, leftStartConfiguration);

  auto untimedTrajectory = robot.planToConfiguration(
      rightArmSpace, endEffector.get(), goalConfiguration, planningTimeout);
  if (!untimedTrajectory)
    throw std::runtime_error("Failed to find a solution");

  const auto constraintCenter
    = (startPose.translation() + goalPose.translation()) / 2.;
  auto constraint = std::make_shared<DistanceConstraint>(
    constraintCenter, constraintRadius);

  aikido::constraint::FrameDifferentiable actualConstraint(
      rightArmSpace, endEffector.get(), constraint);

  const auto velocityLimits = robot.getVelocityLimits(*rightArm);
  const auto accelerationLimits = robot.getAccelerationLimits(*rightArm);
  const auto constraintTolerance = 0.05;
  const auto interpolationTimestep = 0.05;

  std::shared_ptr<Spline> mintosTrajectory =
      interpolateAndTimeOptimizeTrajectory(
          *untimedTrajectory, actualConstraint, -velocityLimits, velocityLimits,
          -accelerationLimits, accelerationLimits, constraintTolerance,
          interpolationTimestep);
  auto parabolicTrajectory =
      robot.retimeTrajectory(rightArmSpace, untimedTrajectory);

  // TODO: MinTOS leaves the robot in a different configuration.
  robot.setConfiguration(rightArmSpace, rightStartConfiguration);

  ros::init(argc, argv, "ex05_mintos");

  InteractiveMarkerViewer viewer(topicName);
  viewer.addSkeleton(robot.getSkeleton());
  viewer.addFrame(startFrame.get(), 0.25, 0.02, 0.5);
  viewer.addFrame(goalFrame.get(), 0.25, 0.02, 0.5);
  viewer.addFrame(endEffectorFrame.get(), 0.25, 0.02, 1.0);

  Eigen::Isometry3d constraintCenterPose = Eigen::Isometry3d::Identity();
  constraintCenterPose.pretranslate(constraintCenter);
  const auto constraintFrame = std::make_shared<SimpleFrame>(
    Frame::World(), "constraint", constraintCenterPose);
  constraintFrame->setShape(
    std::make_shared<dart::dynamics::EllipsoidShape>(
      Eigen::Vector3d::Constant(constraintRadius)));
  constraintFrame->getVisualAspect(true)->setRGBA(
    Eigen::Vector4d(1., 0., 0., 0.5));

  aikido::rviz::ShapeFrameMarker constraintMarker(
    nullptr, &viewer.marker_server(), "constraint", constraintFrame.get());
  constraintMarker.update();

  viewer.setAutoUpdate(true);

  std::cout
    << "ParabolicTimer duration: " << parabolicTrajectory->getDuration() << "\n"
    << "MinTOS duration: " << mintosTrajectory->getDuration() << "\n"
    << "Press <ENTER> to execute." << std::endl;
  std::cin.get();
  //ros::WallDuration(5).sleep();

  robot.execute(rightArmSpace, mintosTrajectory);

  ros::WallDuration(2).sleep();
  robot.execute(rightArmSpace, parabolicTrajectory);

  std::cout << "Press <Ctrl> + C to exit." << std::endl;
  ros::spin();
}
