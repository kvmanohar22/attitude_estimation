#ifndef _ATT_EST_VIZ_H_
#define _ATT_EST_VIZ_H_

#include "attitude_estimation/global.h"
#include "attitude_estimation/utils.h"
#include "ros/message_traits.h"
#include "ros/node_handle.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseStamped.h"

namespace att_est
{

/// type of attitude parameterization
enum class AttitudeType
{
  EULER,
  DCM,
  QUAT
};

class Visualizer
{
public:
  Visualizer(ros::NodeHandle& nh)
    : nh_("~"),
      seq_id_(0)
  {
    pose_pub_dcm_   = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("body_pose_dcm", 10);
    pose_pub_euler_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("body_pose_euler", 10);
    pose_pub_quat_  = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("body_pose_quat", 10);
  }

  virtual ~Visualizer() {}

  void publishPose(
      const double time,
      const Eigen::Quaterniond& q,
      const Matrix3d& cov,
      const AttitudeType type)
  {
    ros::Time stamp; stamp.fromSec(time);
    std_msgs::Header header_msg;
    header_msg.stamp = stamp;
    header_msg.seq = seq_id_;

    geometry_msgs::PoseStamped msg;
    msg.header = header_msg;
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
    msg.pose.position.y = 0.0;
    msg.pose.position.z = 0.0;

    switch(type)
    {
      case AttitudeType::DCM:
        {
          msg.pose.position.x = -4.0;
          header_msg.frame_id = "dcm_body";
          pose_pub_dcm_.publish(msg); 
          break; 
        }
      case AttitudeType::EULER:
        {
          msg.pose.position.x = 0.0;
          header_msg.frame_id = "euler_body";
          pose_pub_euler_.publish(msg); 
          break; 
        }
      case AttitudeType::QUAT:
        {
          msg.pose.position.x = 0.0;
          header_msg.frame_id = "quat_body";
          pose_pub_quat_.publish(msg); 
          break; 
        }
      }
  }

  void publishPoseAll(
      const double time,
      const Matrix3d& R, const Matrix3d& R_cov,     /* output from DCM */
      const Vector3d& phi, const Matrix3d& phi_cov, /* outupt from euler angles */
      const Eigen::Quaterniond& quat)               /* output from quaternions */
  {
    ++seq_id_;

    // DCM to quaternion
    Eigen::Quaterniond q_dcm(R);
    
    // convert euler angles to quaternion (ZYX convention)
    Matrix3d R_euler = rz(-phi.z()) * ry(-phi.y()) * rx(-phi.x());
    Eigen::Quaterniond q_euler(R_euler);

    Matrix3d q_cov; q_cov.setIdentity();
    publishPose(time, q_dcm, q_cov, AttitudeType::DCM);
    publishPose(time, quat, R_cov, AttitudeType::QUAT);
    publishPose(time, q_euler, phi_cov, AttitudeType::EULER);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher  pose_pub_dcm_;
  ros::Publisher  pose_pub_euler_;
  ros::Publisher  pose_pub_quat_;
  size_t seq_id_;
}; // class Visualizer


} // namespace att_est

#endif // _ATT_EST_VIZ_H_
