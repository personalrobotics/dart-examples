#include <chrono>
#include <aikido/control/ros/RosJointStateClient.hpp>
#include <aikido/control/ros/RosTrajectoryExecutor.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <aikido/util/CatkinResourceRetriever.hpp>
#include <aikido/util/ExecutorMultiplexer.hpp>
#include <aikido/util/ExecutorThread.hpp>
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <ros/ros.h>

using aikido::util::ExecutorMultiplexer;
using aikido::util::ExecutorThread;

//=============================================================================
int main(int argc, char** argv)
{
  using aikido::control::ros::RosJointStateClient;
  using aikido::control::ros::RosTrajectoryExecutor;
  using aikido::statespace::dart::MetaSkeletonStateSpace;
  using aikido::rviz::InteractiveMarkerViewer;
  using dart::dynamics::Group;
  using dart::dynamics::Joint;
  using dart::utils::DartLoader;

  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using SplineTrajectory = aikido::trajectory::Spline;

  static const std::string markerTopic{"dart_markers"};
  static const std::string jointStateTopic{"joint_states"};
  static const std::string urdfUri{
    "package://val_description/model/urdf/valkyrie_D.urdf"};
#if 0
  static const std::string topicName{
    "/joint_trajectory_position_controller/follow_joint_trajectory"};
  static const std::vector<std::string> jointNames{
    "rightForearmYaw", "rightWristRoll", "rightWristPitch"};
#else
  static const std::string trajectoryTopicNamespace{
    "/trajectory_controller/follow_joint_trajectory"};
  static const std::vector<std::string> jointNames{
    "rightShoulderPitch", "rightShoulderRoll", "rightShoulderYaw",
    "rightElbowPitch", "rightForearmYaw", "rightWristRoll", "rightWristPitch"};
  static const Vector7d goalPosition = Vector7d::Zero();
#endif
  static const std::chrono::milliseconds controlPeriod{20};
  static const double timestep{0.05};
  static const double goalTimeTolerance{0.5};
  static const double startupTimeTolerance{1.0};
  static const double trajectoryDuration{5.};
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

  ROS_INFO_STREAM("Creating MetaSkeleton.");
  std::vector<Joint*> metaSkeletonJoints;
  for (const auto& jointName : jointNames)
    metaSkeletonJoints.emplace_back(skeleton->getJoint(jointName));

  const auto metaSkeleton = Group::create("RightShoulder");
  metaSkeleton->addJoints(metaSkeletonJoints, true, true);

  ROS_INFO_STREAM("Creating viewer.");
  InteractiveMarkerViewer viewer{markerTopic};
  viewer.addSkeleton(skeleton);
  viewer.setAutoUpdate(true);
  // TODO: Create another Skeleton to visualize the desired position.

  ROS_INFO_STREAM("Creating JointState client.");
  RosJointStateClient jointStateClient{skeleton, nh, jointStateTopic,
    jointStateQueueLength};
  executorCallback.addCallback(
    std::bind(&RosJointStateClient::spin, &jointStateClient));

  usleep(1e6); // TODO: Replace getLatestPosition with a blocking call.
  const auto startPosition = jointStateClient.getLatestPosition(*metaSkeleton);
  metaSkeleton->setPositions(startPosition);

  ROS_INFO_STREAM("Creating TrajectoryExecutor client.");
  RosTrajectoryExecutor trajectoryClient{metaSkeleton, nh,
    trajectoryTopicNamespace, timestep, goalTimeTolerance};
  executorCallback.addCallback(
    std::bind(&RosTrajectoryExecutor::spin, &trajectoryClient));

  ROS_INFO_STREAM("Creating SplineTrajectory.");
  const auto stateSpace = std::make_shared<MetaSkeletonStateSpace>(metaSkeleton);
  const auto trajectory = std::make_shared<SplineTrajectory>(stateSpace);

  Eigen::MatrixXd coefficients{metaSkeleton->getNumDofs(), 2};
  coefficients.col(0) = startPosition;
  coefficients.col(1) = (goalPosition - startPosition) / trajectoryDuration;

  auto state = stateSpace->createState();
  trajectory->addSegment(coefficients, trajectoryDuration, state);

  ROS_INFO_STREAM("Executing trajectory.");
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
