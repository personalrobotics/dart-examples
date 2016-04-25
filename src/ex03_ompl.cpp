#include <iostream>
#include <dart/dart.h>
#include <aikido/constraint/CyclicSampleable.hpp>
#include <aikido/constraint/FrameTestable.hpp>
#include <aikido/constraint/InverseKinematicsSampleable.hpp>
#include <aikido/constraint/JointStateSpaceHelpers.hpp>
#include <aikido/constraint/NonColliding.hpp>
#include <aikido/constraint/TSR.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/planner/ompl/Planner.hpp>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/util/CatkinResourceRetriever.hpp>
#include <aikido/util/RNG.hpp>
#include <aikido/util/StepSequence.hpp>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

using aikido::constraint::CyclicSampleable;
using aikido::constraint::FrameTestable;
using aikido::constraint::InverseKinematicsSampleable;
using aikido::constraint::NonColliding;
using aikido::constraint::TSR;
using aikido::constraint::createProjectableBounds;
using aikido::constraint::createSampleableBounds;
using aikido::constraint::createTestableBounds;
using aikido::distance::createDistanceMetric;
using aikido::planner::ompl::planOMPL;
using aikido::planner::parabolic::computeParabolicTiming;
using aikido::rviz::InteractiveMarkerViewer;
using aikido::statespace::GeodesicInterpolator;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::trajectory::InterpolatedPtr;
using aikido::trajectory::TrajectoryPtr;
using aikido::util::CatkinResourceRetriever;
using aikido::util::RNG;
using aikido::util::RNGWrapper;
using aikido::util::splitEngine;
using dart::collision::FCLCollisionDetector;
using dart::collision::BodyNodeCollisionFilter;
using dart::common::Uri;
using dart::common::make_unique;
using dart::dynamics::Chain;
using dart::dynamics::InverseKinematics;
using dart::dynamics::MetaSkeleton;

static const Uri herbUri { "package://herb_description/robots/herb.urdf" };
static const std::string topicName { "dart_markers" };
static const double planningTimeout { 10. };
static const int maxNumIkTrials { 10 };
static const double maxDistBtwValidityChecks { 0.1 };

namespace {

// TODO: Why is getXXXLowerLimit not const?

Eigen::VectorXd getVelocityLimits(MetaSkeleton& _metaSkeleton)
{
  Eigen::VectorXd velocityLimits(_metaSkeleton.getNumDofs());

  for (size_t i = 0; i < velocityLimits.size(); ++i)
  {
    velocityLimits[i] = std::min(
      -_metaSkeleton.getVelocityLowerLimit(i),
      +_metaSkeleton.getVelocityUpperLimit(i)
    );
    // TODO: Warn if assymmetric.
  }

  return velocityLimits;
}

Eigen::VectorXd getAccelerationLimits(MetaSkeleton& _metaSkeleton)
{
  Eigen::VectorXd accelerationLimits(_metaSkeleton.getNumDofs());

  for (size_t i = 0; i < accelerationLimits.size(); ++i)
  {
      accelerationLimits[i] = std::min(2.0, std::min(
      -_metaSkeleton.getAccelerationLowerLimit(i),
      +_metaSkeleton.getAccelerationUpperLimit(i)
                                           ));

    // TODO: Warn if assymmetric.
  }

  return accelerationLimits;
}

} // namespace

int main(int argc, char** argv)
{
  dart::utils::DartLoader urdfLoader;

  auto resourceRetriever = std::make_shared<CatkinResourceRetriever>();
  auto skeleton = urdfLoader.parseSkeleton(herbUri, resourceRetriever);
  skeleton->enableSelfCollision(false);

  // TODO: Load this from HERB's SRDF file.
  std::vector<std::string> disableCollisions {
    "/left/hand_base",
    "/right/hand_base",
    "/left/wam1",
    "/right/wam1",
    "/left/wam6",
    "/right/wam6"
  };
  for (const auto& bodyNodeName : disableCollisions)
    skeleton->getBodyNode(bodyNodeName)->setCollidable(false);

  auto rightArmBase = skeleton->getBodyNode("/right/wam_base");
  auto rightArmTip = skeleton->getBodyNode("/right/wam7");
  auto rightArm = Chain::create(rightArmBase, rightArmTip, "right_arm");

  auto rightArmIk = InverseKinematics::create(rightArmTip);
  rightArmIk->setDofs(rightArm->getDofs());

  auto collisionDetector = FCLCollisionDetector::create();
  collisionDetector->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);

  auto rightArmSpace = std::make_shared<MetaSkeletonStateSpace>(rightArm);
  auto nonCollidingConstraint = std::make_shared<NonColliding>(
    rightArmSpace, collisionDetector);
  auto collisionGroup = collisionDetector->createCollisionGroupAsSharedPtr(skeleton.get());
  nonCollidingConstraint->addSelfCheck(collisionGroup);

  std::random_device randomDevice;
  auto seedEngine = RNGWrapper<std::default_random_engine>(randomDevice());
  auto engines = splitEngine(seedEngine, 3);

  auto startState = rightArmSpace->getScopedStateFromMetaSkeleton();

  // Goal region - point TSR around relaxed home
  auto goalState = rightArmSpace->createState();
  Eigen::VectorXd goalConfiguration(7);
  goalConfiguration << 5.65, -1.76, -0.26,  1.96, -1.15 , 0.87, -1.43; // relaxed home
  rightArmSpace->convertPositionsToState(goalConfiguration, goalState);
  rightArmSpace->setState(goalState);
  auto goalRegion = std::make_shared<TSR>(std::move(engines[0]), rightArmTip->getTransform());

  auto untimedTrajectory = planOMPL<ompl::geometric::RRTConnect>(
    startState,
    std::make_shared<FrameTestable>(rightArmSpace, rightArmTip, goalRegion),
    std::make_shared<InverseKinematicsSampleable>(
      rightArmSpace,
      std::make_shared<CyclicSampleable>(goalRegion),
      createSampleableBounds(rightArmSpace, std::move(engines[1])),
      rightArmIk,
      maxNumIkTrials
    ),
    rightArmSpace,
    std::make_shared<GeodesicInterpolator>(rightArmSpace),
    createDistanceMetric(rightArmSpace),
    createSampleableBounds(rightArmSpace, std::move(engines[2])),
    nonCollidingConstraint,
    createTestableBounds(rightArmSpace),
    createProjectableBounds(rightArmSpace),
    planningTimeout,
    maxDistBtwValidityChecks
  );
  if (!untimedTrajectory)
    throw std::runtime_error("Planning failed.");

  auto timedTrajectory = computeParabolicTiming(
    *untimedTrajectory,
    getVelocityLimits(*rightArm),
    getAccelerationLimits(*rightArm));
  
  // TODO: Simulate execution.

  ros::init(argc, argv, "ex03_ompl");

  InteractiveMarkerViewer viewer(topicName);
  viewer.addSkeleton(skeleton);
  viewer.setAutoUpdate(true);

  // TODO: Remove in favor of actual simulated execution
  std::cout << "Playing back untimed trajectory..." << std::endl;
  double stepsize = 0.05;
  aikido::util::StepSequence seq(stepsize, true, timedTrajectory->getStartTime(),
                                 timedTrajectory->getEndTime());
  auto state = rightArmSpace->createState();

  for( double t: seq ){
      timedTrajectory->evaluate(t, state);
      rightArmSpace->setState(state);

      usleep(stepsize*1000*1000);
  }

  std::cout << "Press <Ctrl> + C to exit." << std::endl;
  ros::spin();
}
