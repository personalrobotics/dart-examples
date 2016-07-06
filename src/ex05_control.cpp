#include <chrono>
#include <aikido/control/ros/RosJointStateClient.hpp>
#include <aikido/control/ros/RosTrajectoryExecutor.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <aikido/util/CatkinResourceRetriever.hpp>
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <ros/ros.h>

//=============================================================================
class ExecutorThread final
{
public:
  ExecutorThread(
    std::function<void ()> _callback,
    std::chrono::milliseconds _period);

  ~ExecutorThread();

  bool is_running() const;

  void stop();

private:
  void spin();

  std::function<void ()> mCallback;
  std::chrono::milliseconds mPeriod;
  std::atomic<bool> mIsRunning;
  std::thread mThread;
};

//=============================================================================
ExecutorThread::ExecutorThread(
      std::function<void ()> _callback,
      std::chrono::milliseconds _period)
  : mCallback{std::move(_callback)}
  , mPeriod{_period}
  , mIsRunning{true}
{
  mThread = std::thread(&ExecutorThread::spin, this);
}

//=============================================================================
ExecutorThread::~ExecutorThread()
{
  stop();
}

//=============================================================================
bool ExecutorThread::is_running() const
{
  return mIsRunning.load();
}

//=============================================================================
void ExecutorThread::stop()
{
  ROS_INFO("Stopping executor thread.");
  mIsRunning.store(false);
  mThread.join();
}

//=============================================================================
void ExecutorThread::spin()
{
  auto currentTime = std::chrono::steady_clock::now();

  while (mIsRunning.load())
  {
    mCallback();

    currentTime += mPeriod;
    std::this_thread::sleep_until(currentTime);
  }

  ROS_INFO("Exiting executor thread.");
}

//=============================================================================
class ExecutorMultiplexer final
{
public:
  ExecutorMultiplexer() = default;

  void addCallback(std::function<void ()> _callback);

  void operator ()();

private:
  std::mutex mMutex;
  std::vector<std::function<void ()>> mCallbacks;
};

//=============================================================================
void ExecutorMultiplexer::addCallback(std::function<void ()> _callback)
{
  std::lock_guard<std::mutex> lock{mMutex};
  mCallbacks.emplace_back(std::move(_callback));
}

//=============================================================================
void ExecutorMultiplexer::operator ()()
{
  std::lock_guard<std::mutex> lock{mMutex};

  for (auto& callback : mCallbacks)
    callback();
}

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
    "joint_trajectory_effort_controller/follow_joint_trajectory"};
  static const std::vector<std::string> jointNames{
    "rightShoulderPitch", "rightShoulderRoll", "rightShoulderYaw",
    "rightElbowPitch"};
  static const Eigen::Vector4d trajectoryEndpoint{
    0.161101, 0.214281, 0.2127471, 1.419200};
#endif
  static const std::chrono::milliseconds controlPeriod{20};
  static const double timestep{0.05};
  static const double goalTimeTolerance{0.5};
  static const double startupTimeTolerance{1.0};
  static const double trajectoryDuration{5.};

  ROS_INFO("Starting ROS node.");
  ros::init(argc, argv, "ex05_control");
  ros::NodeHandle nh;
  
  if (!ros::Time::waitForValid(ros::WallDuration(startupTimeTolerance)))
    throw std::runtime_error("Timed out while waiting for valid ros::Time.");

  ROS_INFO("Starting execution thread.");
  ExecutorMultiplexer executorCallback;
  ExecutorThread executorThread{std::ref(executorCallback), controlPeriod};

  ROS_INFO("Loading URDF from: %s", urdfUri.c_str());
  dart::utils::DartLoader urdfLoader;
  const auto resourceRetriever
    = std::make_shared<aikido::util::CatkinResourceRetriever>();
  const auto skeleton = urdfLoader.parseSkeleton(urdfUri, resourceRetriever);

  ROS_INFO("Creating MetaSkeleton.");
  std::vector<Joint*> metaSkeletonJoints;
  for (const auto& jointName : jointNames)
    metaSkeletonJoints.emplace_back(skeleton->getJoint(jointName));

  const auto metaSkeleton = Group::create("RightShoulder");
  metaSkeleton->addJoints(metaSkeletonJoints, true, true);

  ROS_INFO("Creating viewer.");
  InteractiveMarkerViewer viewer{markerTopic};
  viewer.addSkeleton(skeleton);
  viewer.setAutoUpdate(true);

  ROS_INFO("Creating JointState client.");
  RosJointStateClient jointStateClient{skeleton, nh, jointStateTopic};
  executorCallback.addCallback(
    std::bind(&RosJointStateClient::spin, &jointStateClient));

  ROS_INFO("Creating TrajectoryExecutor client.");
  RosTrajectoryExecutor trajectoryClient{metaSkeleton, nh,
    trajectoryTopicNamespace, timestep, goalTimeTolerance};
  executorCallback.addCallback(
    std::bind(&RosTrajectoryExecutor::spin, &trajectoryClient));

  ROS_INFO("Creating SplineTrajectory.");
  const auto stateSpace = std::make_shared<MetaSkeletonStateSpace>(metaSkeleton);
  const auto trajectory = std::make_shared<SplineTrajectory>(stateSpace);

  Eigen::MatrixXd coefficients{metaSkeleton->getNumDofs(), 2};
  coefficients.col(0) = trajectoryEndpoint;
  coefficients.col(1).setZero();

  auto state = stateSpace->createState();
  trajectory->addSegment(coefficients, trajectoryDuration, state);

  ROS_INFO("Executing trajectory.");
  auto trajectoryFuture = trajectoryClient.execute(trajectory);

  ROS_INFO("Waiting for trajectory to finish.");
  trajectoryFuture.wait();

  try
  {
    trajectoryFuture.get();
    ROS_INFO("Execution succeeded.");
  }
  catch (const std::runtime_error& e)
  {
    ROS_ERROR("%s", e.what());
  }

  ROS_INFO("Done. Press <Ctrl> + C to exit.");
  ros::spin();
  ROS_INFO("Exiting.");

  return 0;
}
