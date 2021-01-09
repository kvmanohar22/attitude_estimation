#ifndef _ATT_EST_VIZ_H_
#define _ATT_EST_VIZ_H_

#include "attitude_estimation/global.h"
#include "ros/message_traits.h"
#include "ros/node_handle.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseStamped.h"

namespace att_est
{

class Visualizer
{
public:
  Visualizer(ros::NodeHandle& nh)
    : nh_("~"),
      seq_id_(0)
  {
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("body_pose", 10); 
  }

  virtual ~Visualizer() {}

  void publishPose(const double time, const Matrix3d& R)
  {
    ++seq_id_;
    ros::Time stamp; stamp.fromSec(time);
    std_msgs::Header header_msg;
    header_msg.frame_id = "/body";
    header_msg.stamp = stamp;
    header_msg.seq = seq_id_;

    Eigen::Quaterniond q(R);
    geometry_msgs::PoseStamped msg;
    msg.header = header_msg;
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
    msg.pose.position.x = 0.0;
    msg.pose.position.y = 0.0;
    msg.pose.position.z = 0.0;
 
    pose_pub_.publish(msg);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher  pose_pub_;
  size_t seq_id_;
}; // class Visualizer


} // namespace att_est

#endif // _ATT_EST_VIZ_H_
