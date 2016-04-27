#include "herb.hpp"
#include <aikido/constraint/JointStateSpaceHelpers.hpp>
#include <aikido/constraint/TSR.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/planner/ompl/Planner.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/util/CatkinResourceRetriever.hpp>
#include <aikido/util/RNG.hpp>
#include <aikido/util/StepSequence.hpp>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

using aikido::constraint::NonColliding;
using aikido::constraint::TSR;
using aikido::constraint::createProjectableBounds;
using aikido::constraint::createSampleableBounds;
using aikido::constraint::createTestableBounds;
using aikido::distance::createDistanceMetric;
using aikido::planner::ompl::planOMPL;
using aikido::planner::parabolic::computeParabolicTiming;
using aikido::statespace::GeodesicInterpolator;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using aikido::trajectory::InterpolatedPtr;
using aikido::trajectory::TrajectoryPtr;
using aikido::util::CatkinResourceRetriever;
using aikido::util::RNGWrapper;
using aikido::util::StepSequence;
using dart::collision::FCLCollisionDetector;
using dart::common::make_unique;
using dart::dynamics::Chain;
using dart::dynamics::ChainPtr;
using dart::dynamics::InverseKinematics;
using dart::dynamics::MetaSkeleton;
using dart::dynamics::SkeletonPtr;


Herb::Herb() : mCollisionResolution(0.02) {

  dart::utils::DartLoader urdfLoader;

  auto resourceRetriever = std::make_shared<CatkinResourceRetriever>();
  mRobot = urdfLoader.parseSkeleton(herbUri, resourceRetriever);

  auto rightArmBase = mRobot->getBodyNode("/right/wam_base");
  mRightEndEffector = mRobot->getBodyNode("/right/wam7");
  mRightArm = Chain::create(rightArmBase, mRightEndEffector, "right_arm");

  auto rightArmIk = InverseKinematics::create(mRightEndEffector);
  rightArmIk->setDofs(mRightArm->getDofs());
}

SkeletonPtr Herb::getSkeleton() const { return mRobot; }

ChainPtr Herb::getRightArm() const { return mRightArm; }

Eigen::VectorXd Herb::getVelocityLimits(MetaSkeleton &_metaSkeleton) const {
  Eigen::VectorXd velocityLimits(_metaSkeleton.getNumDofs());

  for (size_t i = 0; i < velocityLimits.size(); ++i) {
    velocityLimits[i] = std::min(-_metaSkeleton.getVelocityLowerLimit(i),
                                 +_metaSkeleton.getVelocityUpperLimit(i));
    // TODO: Warn if assymmetric.
  }

  return velocityLimits;
}

Eigen::VectorXd Herb::getAccelerationLimits(MetaSkeleton &_metaSkeleton) const {
  Eigen::VectorXd accelerationLimits(_metaSkeleton.getNumDofs());

  for (size_t i = 0; i < accelerationLimits.size(); ++i) {
    accelerationLimits[i] = std::min(2.0, 
        std::min(-_metaSkeleton.getAccelerationLowerLimit(i),
                 +_metaSkeleton.getAccelerationUpperLimit(i)));
    // TODO: Warn if assymmetric.
  }

  return accelerationLimits;
}

void Herb::setConfiguration(MetaSkeletonStateSpacePtr _space,
                            const Eigen::VectorXd &_configuration) {
  auto state = _space->createState();
  _space->convertPositionsToState(_configuration, state);
  _space->setState(state);
}

InterpolatedPtr Herb::planToConfiguration(MetaSkeletonStateSpacePtr _space,
                                          const Eigen::VectorXd &_goal,
                                          double _timelimit) const {
  auto startState = _space->getScopedStateFromMetaSkeleton();
  auto goalState = _space->createState();
  _space->convertPositionsToState(_goal, goalState);

  auto rng = make_unique<RNGWrapper<std::default_random_engine>>(0);

  auto untimedTrajectory = planOMPL<ompl::geometric::RRTConnect>(
      startState,
      goalState,
      _space,
      std::make_shared<GeodesicInterpolator>(_space),
      createDistanceMetric(_space),
      createSampleableBounds(_space, std::move(rng)),
      getSelfCollisionConstraint(_space),
      createTestableBounds(_space),
      createProjectableBounds(_space),
      _timelimit, mCollisionResolution
    );

  return untimedTrajectory;
}

TrajectoryPtr Herb::retimeTrajectory(MetaSkeletonStateSpacePtr _space,
                                 InterpolatedPtr _traj)
{
  auto skel = _space->getMetaSkeleton();
  return computeParabolicTiming(
      *_traj,
      getVelocityLimits(*skel),
      getAccelerationLimits(*skel));
}

void Herb::execute(MetaSkeletonStateSpacePtr _space,
                   TrajectoryPtr _traj)
{
  // TODO: Remove in favor of actual simulated execution
  double stepsize = 0.05;
  aikido::util::StepSequence seq(stepsize, true, _traj->getStartTime(),
                                 _traj->getEndTime());
  auto state = _space->createState();

  for( double t: seq ){
      _traj->evaluate(t, state);
      _space->setState(state);
      usleep(stepsize*1000*1000);
  }
}

std::shared_ptr<NonColliding>
Herb::getSelfCollisionConstraint(MetaSkeletonStateSpacePtr _space) const {
  mRobot->enableSelfCollision(false);

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
    mRobot->getBodyNode(bodyNodeName)->setCollidable(false);

  auto collisionDetector = FCLCollisionDetector::create();
  collisionDetector->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  auto nonCollidingConstraint =
      std::make_shared<NonColliding>(_space, collisionDetector);
  nonCollidingConstraint->addSelfCheck(
      collisionDetector->createCollisionGroupAsSharedPtr(
          mRobot.get()));
  return nonCollidingConstraint;
}

