#include <chrono>
#include <aikido/control/ros/RosJointStateClient.hpp>
#include <aikido/control/ros/RosTrajectoryExecutor.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <aikido/rviz/SkeletonMarker.hpp>
#include <aikido/util/CatkinResourceRetriever.hpp>
#include <aikido/util/ExecutorMultiplexer.hpp>
#include <aikido/util/ExecutorThread.hpp>
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <ros/ros.h>

using control_msgs::FollowJointTrajectoryActionFeedback;

class FeedbackCallback
{
public:
  FeedbackCallback(
        dart::dynamics::MetaSkeletonPtr desiredSkeleton,
        dart::dynamics::MetaSkeletonPtr actualSkeleton)
    : mDesiredSkeleton{desiredSkeleton}
    , mActualSkeleton{actualSkeleton}
  {
  }

  void operator ()(const FollowJointTrajectoryActionFeedback& feedbackMsg)
  {
    const auto& feedback = feedbackMsg.feedback;

    if (mDesiredSkeleton)
    {
      const Eigen::Map<const Eigen::VectorXd> desiredPositions(
        feedback.desired.positions.data(), feedback.desired.positions.size());
      mDesiredSkeleton->setPositions(desiredPositions);
    }

    if (mActualSkeleton)
    {
      const Eigen::Map<const Eigen::VectorXd> actualPositions(
        feedback.actual.positions.data(), feedback.actual.positions.size());
      mActualSkeleton->setPositions(actualPositions);
    }
  };

private:
  dart::dynamics::MetaSkeletonPtr mDesiredSkeleton;
  dart::dynamics::MetaSkeletonPtr mActualSkeleton;
};

//=============================================================================
int main(int argc, char** argv)
{
  using aikido::control::ros::RosJointStateClient;
  using aikido::control::ros::RosTrajectoryExecutor;
  using aikido::planner::parabolic::computeParabolicTiming;
  using aikido::rviz::InteractiveMarkerViewer;
  using aikido::statespace::GeodesicInterpolator;
  using aikido::statespace::dart::MetaSkeletonStateSpace;
  using aikido::util::ExecutorMultiplexer;
  using aikido::util::ExecutorThread;
  using dart::dynamics::Group;
  using dart::dynamics::Joint;
  using dart::utils::DartLoader;

  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using InterpolatedTrajectory = aikido::trajectory::Interpolated;
  using SplineTrajectory = aikido::trajectory::Spline;

  static const std::string markerTopic{"dart_markers"};
  static const std::string jointStateTopic{"joint_states"};
  static const std::string urdfUri{
    "package://val_description/model/urdf/valkyrie_D.urdf"};
  static const std::string trajectoryTopicNamespace{
    "/trajectory_controller/follow_joint_trajectory"};
  static const std::vector<std::string> jointNames{
    "rightShoulderPitch", "rightShoulderRoll", "rightShoulderYaw",
    "rightElbowPitch", "rightForearmYaw", "rightWristRoll", "rightWristPitch"};
  static const Vector7d goalPosition
    = (Vector7d() << 0.161101, 0.214281, 0.212747, 1.419200, 0.166835,
                     0.398457, 0.139980).finished();
  static const Vector7d velocityLimit = Vector7d::Constant(0.25); // TODO
  static const Vector7d accelerationLimit = Vector7d::Constant(0.25); // TODO
  static const std::chrono::milliseconds controlPeriod{20};
  static const double timestep{0.05};
  static const double goalTimeTolerance{0.5};
  static const double startupTimeTolerance{1.0};
  static const double trajectoryDuration{2.};
  static const size_t jointStateQueueLength{100};

  ROS_INFO_STREAM("Starting ROS node.");
  ros::init(argc, argv, "ex05_control");
  ros::NodeHandle nh;
  
  if (!ros::Time::waitForValid(ros::WallDuration(startupTimeTolerance)))
    throw std::runtime_error("Timed out while waiting for valid ros::Time.");

  ROS_INFO_STREAM("Starting execution thread.");
  ExecutorMultiplexer executorCallback;
  ExecutorThread executorThread{std::ref(executorCallback), controlPeriod};

  ROS_INFO_STREAM("Loading URDF from: " << urdfUri.c_str());
  dart::utils::DartLoader urdfLoader;
  const auto resourceRetriever
    = std::make_shared<aikido::util::CatkinResourceRetriever>();
  const auto skeleton = urdfLoader.parseSkeleton(urdfUri, resourceRetriever);
  const auto ghostSkeleton = skeleton->clone("GhostSkeleton");

  ROS_INFO_STREAM("Creating MetaSkeleton.");
  std::vector<Joint*> metaSkeletonJoints, ghostMetaSkeletonJoints;
  for (const auto& jointName : jointNames)
  {
    metaSkeletonJoints.emplace_back(skeleton->getJoint(jointName));
    ghostMetaSkeletonJoints.emplace_back(ghostSkeleton->getJoint(jointName));
  }

  const auto metaSkeleton = Group::create("RightShoulder");
  metaSkeleton->addJoints(metaSkeletonJoints, true, true);

  const auto ghostMetaSkeleton = Group::create("RightShoulder");
  ghostMetaSkeleton->addJoints(ghostMetaSkeletonJoints, true, true);

  ROS_INFO_STREAM("Creating JointState client.");
  RosJointStateClient jointStateClient{skeleton, nh, jointStateTopic,
    jointStateQueueLength};
  executorCallback.addCallback(
    std::bind(&RosJointStateClient::spin, &jointStateClient));

  usleep(1e6); // TODO: Replace getLatestPosition with a blocking call.
  const auto startPosition = jointStateClient.getLatestPosition(*metaSkeleton);
  metaSkeleton->setPositions(startPosition);
  ghostMetaSkeleton->setPositions(startPosition);

  ROS_INFO_STREAM("Creating TrajectoryExecutor client.");
  RosTrajectoryExecutor trajectoryClient{metaSkeleton, nh,
    trajectoryTopicNamespace, timestep, goalTimeTolerance};
  executorCallback.addCallback(
    std::bind(&RosTrajectoryExecutor::spin, &trajectoryClient));

  ROS_INFO_STREAM("Creating viewer.");
  InteractiveMarkerViewer viewer{markerTopic};
  viewer.addSkeleton(skeleton);

  const auto ghostSkeletonMarker = viewer.addSkeleton(ghostSkeleton);
  ghostSkeletonMarker->update();
  ghostSkeletonMarker->SetColor(Eigen::Vector4d(1.0, 1.0, 1.0, 0.5));

  viewer.setAutoUpdate(true);
  // TODO: Create another Skeleton to visualize the desired position.
  
  ROS_WARN_STREAM("Creating debug visualizer.");
  const std::string feedbackTopicName{trajectoryTopicNamespace + "/feedback"};
  FeedbackCallback feedbackCallback{ghostMetaSkeleton, metaSkeleton};
  boost::function<void (const FollowJointTrajectoryActionFeedback&)>
    feedbackCallbackWrapper{feedbackCallback};
  const auto feedbackSubscriber
    = nh.subscribe<FollowJointTrajectoryActionFeedback>(
        feedbackTopicName, 1, feedbackCallbackWrapper);
  executorCallback.addCallback(&ros::spinOnce);

  ROS_INFO_STREAM("Creating un-timed InterpolatedTrajectory.");
  const auto stateSpace = std::make_shared<MetaSkeletonStateSpace>(metaSkeleton);
  const auto interpolator = std::make_shared<GeodesicInterpolator>(stateSpace);
  const auto path = std::make_shared<InterpolatedTrajectory>(
    stateSpace, interpolator);
  auto state = stateSpace->createState();

  stateSpace->convertPositionsToState(startPosition, state);
  path->addWaypoint(0., state);

  stateSpace->convertPositionsToState(goalPosition, state);
  path->addWaypoint(1., state);

  ROS_INFO_STREAM("Computing timed SplineTrajectory.");
  const std::shared_ptr<SplineTrajectory> trajectory = computeParabolicTiming(
    *path, velocityLimit, accelerationLimit);

  std::cout << "Press <ENTER> to execute." << std::endl;
  std::cin.get();

  ROS_INFO_STREAM("Executing trajectory with duration "
    << trajectory->getDuration() << ".");
  auto trajectoryFuture = trajectoryClient.execute(trajectory);

  ROS_INFO_STREAM("Waiting for trajectory to finish.");
  trajectoryFuture.wait();

  try
  {
    trajectoryFuture.get();
    ROS_INFO_STREAM("Execution succeeded.");
  }
  catch (const std::runtime_error& e)
  {
    ROS_ERROR("%s", e.what());
  }

  ROS_INFO_STREAM("Done. Press <Ctrl> + C to exit.");
  ros::spin();
  ROS_INFO_STREAM("Exiting.");

  return 0;
}
