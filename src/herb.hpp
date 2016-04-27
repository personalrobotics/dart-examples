#ifndef DARTEXAMPLES_HERB_HPP_
#define DARTEXAMPLES_HERB_HPP_

#include <dart/dart.h>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/Trajectory.hpp>
#include <aikido/constraint/NonColliding.hpp>

class Herb {

public:
  const dart::common::Uri herbUri{
      "package://herb_description/robots/herb.urdf"};
  ;

  /// Construct the herb metaskeleton
  Herb();

  /// Get the robot
  dart::dynamics::SkeletonPtr getSkeleton() const;

  /// Get the right arm
  dart::dynamics::ChainPtr getRightArm() const;

  /// Compute velocity limits from the MetaSkeleton
  Eigen::VectorXd
  getVelocityLimits(dart::dynamics::MetaSkeleton &_metaSkeleton) const;

  /// Compute acceleration limits from the MetaSkeleton
  Eigen::VectorXd
  getAccelerationLimits(dart::dynamics::MetaSkeleton &_metaSkeleton) const;

  /// Set the configuration of the metaskeleton defined in the given statespace
  /// \param _space The StateSpace for the metaskeleton
  /// \param _configuration The configuration to set
  void
  setConfiguration(aikido::statespace::dart::MetaSkeletonStateSpacePtr _space,
                   const Eigen::VectorXd &_configuration);

  /// Plan the robot to a specific configuration
  aikido::trajectory::InterpolatedPtr planToConfiguration(
      aikido::statespace::dart::MetaSkeletonStateSpacePtr _space,
      const Eigen::VectorXd &_goal, double _timelimit) const;

  /// Retime a trajectory to respect Herb's velocity and acceleration limits
  aikido::trajectory::TrajectoryPtr
  retimeTrajectory(aikido::statespace::dart::MetaSkeletonStateSpacePtr _space,
                   aikido::trajectory::InterpolatedPtr _traj);

  /// Execute a trajectory
  void execute(aikido::statespace::dart::MetaSkeletonStateSpacePtr _space,
               aikido::trajectory::TrajectoryPtr _traj);

protected:
  /// Generate a constraint that is satisfied when the MetaSkeleton is not
  /// in
  /// self collision
  std::shared_ptr<aikido::constraint::NonColliding> getSelfCollisionConstraint(
      aikido::statespace::dart::MetaSkeletonStateSpacePtr _space) const;
private:
  double mCollisionResolution;
  dart::dynamics::SkeletonPtr mRobot;
  dart::dynamics::ChainPtr mRightArm;
  dart::dynamics::BodyNodePtr mRightEndEffector;
};

#endif
