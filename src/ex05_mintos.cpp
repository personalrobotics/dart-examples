#include "herb.hpp"
#include "DistanceConstraint.hpp"
#include <iostream>
#include <fstream>
#include <dart/dart.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <aikido/constraint/JointStateSpaceHelpers.hpp>
#include <aikido/constraint/FrameDifferentiable.hpp>
#include <aikido/constraint/FrameTestable.hpp>
#include <aikido/planner/ompl/CRRTConnect.hpp>
#include <aikido/planner/ompl/Planner.hpp>
#include <aikido/constraint/NewtonsMethodProjectable.hpp>
#include <aikido/constraint/CyclicSampleable.hpp>
#include <aikido/constraint/InverseKinematicsSampleable.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/util/StepSequence.hpp>
#include <aikido/planner/mintos/MinTOS.hpp>
#include <aikido/constraint/TSR.hpp>
#include <aikido/rviz/ShapeFrameMarker.hpp>

using aikido::constraint::CyclicSampleable;
using aikido::rviz::InteractiveMarkerViewer;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::constraint::NewtonsMethodProjectable;
using aikido::constraint::FrameDifferentiable;
using aikido::constraint::InverseKinematicsSampleable;
using aikido::planner::ompl::planCRRTConnect;
using aikido::planner::ompl::planCRRT;
using aikido::constraint::TSR;
using aikido::statespace::GeodesicInterpolator;
using aikido::constraint::FrameTestable;
using aikido::constraint::createSampleableBounds;
using aikido::distance::createDistanceMetric;
using aikido::constraint::createTestableBounds;
using aikido::constraint::createProjectableBounds;
using aikido::planner::mintos::interpolateAndTimeOptimizeTrajectory;
using aikido::trajectory::Spline;
using aikido::util::StepSequence;
using dart::dynamics::Frame;
using dart::dynamics::SimpleFrame;
using aikido::util::RNGWrapper;

static const std::string topicName{"dart_markers"};
static const double planningTimeout{60.};
static const int numIkTrials{10};
static const double constraintTolerance{1e-4};
static const Eigen::Vector3d goalOffset{0., 0.4, 0.};
static const double serializationTimestep{0.01};

static const Eigen::VectorXd velocityLimit
  = Eigen::VectorXd::Constant(7, 2.5);
static const Eigen::VectorXd accelerationLimit
  = Eigen::VectorXd::Constant(7, 2.0);

static void serializeTrajectory(
  const aikido::trajectory::Trajectory& trajectory, double timestep,
  std::ostream& stream)
{
  const Eigen::IOFormat formatCsv(
    Eigen::StreamPrecision, Eigen::DontAlignCols, ",", "\n", "", "", "", "");

  const auto space = trajectory.getStateSpace();
  auto state = space->createState();
  Eigen::VectorXd tangentVector;

  StepSequence timeSequence(
    timestep, true, trajectory.getStartTime(), trajectory.getEndTime());
  for (const auto timeFromStart : timeSequence)
  {
    trajectory.evaluate(timeFromStart, state);
    space->logMap(state, tangentVector);
    stream << timeFromStart << ","
      << tangentVector.transpose().format(formatCsv) << "\n";
  }
}

int main(int argc, char **argv)
{
  Herb robot;

  auto rightArm = robot.getRightArm();

  rightArm->setVelocityLowerLimits(-velocityLimit);
  rightArm->setVelocityUpperLimits( velocityLimit);
  rightArm->setAccelerationLowerLimits(-accelerationLimit);
  rightArm->setAccelerationUpperLimits( accelerationLimit);

  auto rightArmSpace = std::make_shared<MetaSkeletonStateSpace>(rightArm);
  auto endEffector = robot.getRightEndEffector();

  std::random_device randomDevice;
  auto seedEngine = RNGWrapper<std::default_random_engine>(randomDevice());
  auto engines = splitEngine(seedEngine, 3);

  // Hard-coded start configuration.
  Eigen::VectorXd rightStartConfiguration(7);
  rightStartConfiguration
    << 5.65, -1.09011077, -0.26,  1.96, -1.15, 0.81463122, -2.37836581;
  robot.setConfiguration(rightArmSpace, rightStartConfiguration);
  const auto startPose = endEffector->getTransform();

  // Set the left arm into a safe configuration.
  auto leftArm = robot.getLeftArm();
  auto leftArmSpace = std::make_shared<MetaSkeletonStateSpace>(leftArm);
  Eigen::VectorXd leftStartConfiguration(7);
  leftStartConfiguration << 0.64, -1.76, 0.26, 1.96, 1.16, 0.87, 1.43;
  robot.setConfiguration(leftArmSpace, leftStartConfiguration);

  // Compute a goal configuration.
  Eigen::Isometry3d goalPose = endEffector->getTransform();
  goalPose.pretranslate(goalOffset);
  const auto goalPoseConstraint = std::make_shared<TSR>(
    std::move(engines[0]), goalPose);
  const auto goalTestableConstraint = std::make_shared<FrameTestable>(
    rightArmSpace, endEffector.get(), goalPoseConstraint);
  const auto goalSampleableConstraint = std::make_shared<InverseKinematicsSampleable>(
    rightArmSpace,
    // TODO: Is this necessary?
    std::make_shared<CyclicSampleable>(goalPoseConstraint),
    createSampleableBounds(rightArmSpace, std::move(engines[1])),
    endEffector->createIK(), numIkTrials);

  // Construct the trajectory-wide constraint.
  const auto constraintCenter
    = ((startPose.translation() + goalPose.translation()) / 2.).eval();
  const auto constraintRadius
    = ((startPose.translation() - goalPose.translation()).norm()) / 2.;
  const auto eeConstraint  = std::make_shared<DistanceConstraint>(
    constraintCenter, constraintRadius);
  const auto trajectoryDifferentiableConstraint = std::make_shared<FrameDifferentiable>(
    rightArmSpace, endEffector.get(), eeConstraint);
  const auto trajectoryProjectableConstraint = std::make_shared<NewtonsMethodProjectable>(
    trajectoryDifferentiableConstraint, std::vector<double>{constraintTolerance});

  std::cout << "Initializing ROS and viewer..." << std::endl;
  ros::init(argc, argv, "ex05_mintos");

  InteractiveMarkerViewer viewer(topicName);
  viewer.addSkeleton(robot.getSkeleton());

  SimpleFrame startFrame(Frame::World(), "start", startPose);
  viewer.addFrame(&startFrame, 0.25, 0.02, 0.5);

  SimpleFrame goalFrame(Frame::World(), "goal", goalPose);
  viewer.addFrame(&goalFrame, 0.25, 0.02, 0.5);

  SimpleFrame endEffectorFrame(endEffector, "ee", Eigen::Isometry3d::Identity());
  viewer.addFrame(&endEffectorFrame, 0.25, 0.02, 1.0);

  Eigen::Isometry3d constraintCenterPose = Eigen::Isometry3d::Identity();
  constraintCenterPose.pretranslate(constraintCenter);
  SimpleFrame constraintFrame(
    Frame::World(), "constraint", constraintCenterPose);
  constraintFrame.setShape(
    std::make_shared<dart::dynamics::EllipsoidShape>(
      2. * Eigen::Vector3d::Constant(constraintRadius)));
  constraintFrame.getVisualAspect(true)->setRGBA(
    Eigen::Vector4d(1., 0., 0., 0.5));
  aikido::rviz::ShapeFrameMarker constraintMarker(
    nullptr, &viewer.marker_server(), "constraint", &constraintFrame);
  constraintMarker.update();
  viewer.update();

  std::cout << "Planning constrained trajectory." << std::endl;
  const auto startState = rightArmSpace->getScopedStateFromMetaSkeleton();
  const auto untimedTrajectory = planCRRTConnect(
    startState, 
    goalTestableConstraint,
    goalSampleableConstraint,
    trajectoryProjectableConstraint,
    rightArmSpace,
    std::make_shared<GeodesicInterpolator>(rightArmSpace),
    createDistanceMetric(rightArmSpace),
    createSampleableBounds(rightArmSpace, std::move(engines[2])),
    robot.getSelfCollisionConstraint(rightArmSpace), 
    createTestableBounds(rightArmSpace),
    createProjectableBounds(rightArmSpace), 
    planningTimeout,
    std::numeric_limits<double>::infinity(),
    robot.mCollisionResolution,
    robot.mCollisionResolution * 0.5,
    robot.mCollisionResolution
  );
  if (!untimedTrajectory)
    throw std::runtime_error("Failed to find a solution");

  std::cout << "Retiming with ParabolicTimer." << std::endl;
  auto parabolicTrajectory =
      robot.retimeTrajectory(rightArmSpace, untimedTrajectory);

  std::cout << "Retiming with MinTOS..." << std::endl;
  const auto velocityLimits = robot.getVelocityLimits(*rightArm);
  const auto accelerationLimits = robot.getAccelerationLimits(*rightArm);
  const auto constraintTolerance = 0.05;
  const auto interpolationTimestep = 0.05;
  std::shared_ptr<Spline> mintosTrajectory =
      interpolateAndTimeOptimizeTrajectory(
          *untimedTrajectory, *trajectoryDifferentiableConstraint,
          -velocityLimits, velocityLimits,
          -accelerationLimits, accelerationLimits, constraintTolerance,
          interpolationTimestep);

  {
    std::ofstream parabolicOutput("/tmp/parabolic.csv");
    serializeTrajectory(*parabolicTrajectory, serializationTimestep, parabolicOutput);

    std::ofstream mintosOutput("/tmp/mintos.csv");
    serializeTrajectory(*mintosTrajectory, serializationTimestep, mintosOutput);
  }

  std::cout
    << "ParabolicTimer duration: " << parabolicTrajectory->getDuration() << "\n"
    << "MinTOS duration: " << mintosTrajectory->getDuration() << "\n"
    << "Press <ENTER> to execute." << std::endl;
  std::cin.get();

  robot.setConfiguration(rightArmSpace, rightStartConfiguration);
  viewer.setAutoUpdate(true);
  //ros::WallDuration(5).sleep();

  std::cout << "Executing MinTOS trajectory..." << std::endl;
  robot.execute(rightArmSpace, mintosTrajectory);

  ros::WallDuration(2).sleep();
  std::cout << "Executing parabolic trajectory..." << std::endl;
  robot.execute(rightArmSpace, parabolicTrajectory);

  std::cout << "Press <Ctrl> + C to exit." << std::endl;
  ros::spin();
}
