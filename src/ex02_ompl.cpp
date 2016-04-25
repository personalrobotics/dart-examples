#include <iostream>
#include <dart/dart.h>
#include <aikido/constraint/FrameTestable.hpp>
#include <aikido/constraint/InverseKinematicsSampleable.hpp>
#include <aikido/constraint/JointStateSpaceHelpers.hpp>
#include <aikido/constraint/NonColliding.hpp>
#include <aikido/constraint/TSR.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/planner/ompl/Planner.hpp>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/util/CatkinResourceRetriever.hpp>
#include <aikido/util/RNG.hpp>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

using aikido::constraint::FrameTestable;
using aikido::constraint::InverseKinematicsSampleable;
using aikido::constraint::NonColliding;
using aikido::constraint::TSR;
using aikido::constraint::createProjectableBounds;
using aikido::constraint::createSampleableBounds;
using aikido::constraint::createTestableBounds;
using aikido::distance::createDistanceMetric;
using aikido::planner::ompl::planOMPL;
using aikido::rviz::InteractiveMarkerViewer;
using aikido::statespace::GeodesicInterpolator;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::util::CatkinResourceRetriever;
using aikido::util::RNG;
using aikido::util::RNGWrapper;
using aikido::util::splitEngine;
using dart::collision::FCLCollisionDetector;
using dart::common::Uri;
using dart::common::make_unique;
using dart::dynamics::Chain;
using dart::dynamics::InverseKinematics;

static const Uri herbUri { "package://herb_description/robots/herb.urdf" };
static const std::string topicName { "dart_markers" };
static const double planningTimeout { 10. };
static const int maxNumIkTrials { 10 };

int main(int argc, char** argv)
{
  dart::utils::DartLoader urdfLoader;

  auto resourceRetriever = std::make_shared<CatkinResourceRetriever>();
  auto skeleton = urdfLoader.parseSkeleton(herbUri, resourceRetriever);

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
  nonCollidingConstraint->addSelfCheck(
    collisionDetector->createCollisionGroupAsSharedPtr(skeleton.get()));

  auto seedEngine = RNGWrapper<std::default_random_engine>(0);
  auto engines = splitEngine(seedEngine, 3);

  auto startState = rightArmSpace->getScopedStateFromMetaSkeleton();
  auto goalRegion = std::make_shared<TSR>(std::move(engines[0]));

  auto trajectory = planOMPL<ompl::geometric::RRTConnect>(
    startState,
    std::make_shared<FrameTestable>(rightArmSpace, rightArmTip, goalRegion),
    std::make_shared<InverseKinematicsSampleable>(
      rightArmSpace, goalRegion, 
      createSampleableBounds(rightArmSpace, std::move(engines[2])),
      rightArmIk,
      maxNumIkTrials),
    rightArmSpace,
    std::make_shared<GeodesicInterpolator>(rightArmSpace),
    createDistanceMetric(rightArmSpace),
    createSampleableBounds(rightArmSpace, std::move(engines[1])),
    nonCollidingConstraint,
    createTestableBounds(rightArmSpace),
    createProjectableBounds(rightArmSpace),
    planningTimeout
  );

  ros::init(argc, argv, "ex02_ompl");

  InteractiveMarkerViewer viewer(topicName);
  viewer.addSkeleton(skeleton);
  viewer.setAutoUpdate(true);

  std::cout << "Press <Ctrl> + C to exit." << std::endl;
  ros::spin();
}
