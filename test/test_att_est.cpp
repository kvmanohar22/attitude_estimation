#include "attitude_estimation/dcm_estimator.h"
#include "attitude_estimation/gyro.h"
#include "attitude_estimation/utils.h"
#include "attitude_estimation/viz.h"

#include <ios>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/rate.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <fstream>

namespace att_est
{

class TestAttEstimation
{
public:
  TestAttEstimation(ros::NodeHandle& nh);
  virtual ~TestAttEstimation();

  // ROS callback function for imu messages
  void imuCb(const sensor_msgs::Imu::ConstPtr& msg);

  inline bool quit() const { return quit_; }
  inline ros::Rate rate() const { return rate_; }

  void writeLog(const double t, Vector3d& rpy, Matrix3d& P);

private:
  ros::NodeHandle    nh_;
  ros::Subscriber    imu_sub_;
  bool               quit_;
  ros::Rate          rate_;
  std::ofstream      ofs_;
  bool               start_;
  AbstractEstimator* dcm_estimator_;
  Visualizer         viz_;
}; // class TestInsInitAccObs

TestAttEstimation::TestAttEstimation(ros::NodeHandle& nh)
  : nh_(nh),
    quit_(false),
    rate_(1000),
    start_(false),
    viz_(nh)
{
  std::string rostopic; 
  ros::param::get("/attitude_estimation/rostopic", rostopic);
  imu_sub_ = nh_.subscribe(rostopic.c_str(), 1000, &TestAttEstimation::imuCb, this);
  
  // read gyro noise terms
  double gyro_noise_density;
  ros::param::get("/attitude_estimation/gyroscope_noise_density", gyro_noise_density);  
  double gyro_noise_density_sq = gyro_noise_density*gyro_noise_density; 
  Vector3d gyro_noise(gyro_noise_density_sq, gyro_noise_density_sq, gyro_noise_density_sq);
  double update_rate;
  ros::param::get("/attitude_estimation/update_rate", update_rate);
  double dt = 1.0/update_rate;
  dcm_estimator_ = new DcmEstimator(gyro_noise, dt);

  // open file for writing debug data and add header
  ofs_.open("/tmp/att_est.csv");
  ofs_ << "time" << ","
       << "cov00" << "," << "cov11" << ","<< "cov22" << ","
       << "roll" << "," << "pitch" << "," << "yaw" << "\n";
}

void TestAttEstimation::writeLog(const double t, Vector3d& rpy, Matrix3d& P)
{
  // convert to degrees
  rpy = rpy * 180/PI;
  P(0,0) = sqrt(P(0,0)) * 180 / PI;
  P(1,1) = sqrt(P(1,1)) * 180 / PI;
  P(2,2) = sqrt(P(2,2)) * 180 / PI;

  ofs_ << t << ","
       << P(0,0) << "," << P(1,1) << ","<< P(2,2) << ","
       << rpy[0] << "," << rpy[1] << ","<< rpy[2] << "\n";

  ROS_DEBUG_STREAM_THROTTLE(3.0,
      "cov = " << P(0,0) << " " << P(1,1) << " " << P(2,2) << "\t"
      "rpy = " << rpy[0] << " " << rpy[1] << " " << rpy[2]);
}

TestAttEstimation::~TestAttEstimation()
{
  ofs_.close();
}

void TestAttEstimation::imuCb(const sensor_msgs::Imu::ConstPtr& msg)
{
  ROS_INFO_STREAM_ONCE("Imu callback started.");
  if(!start_)
  {
    dcm_estimator_->setInitTime(msg->header.stamp.toSec());
    start_ = true;
    return;
  }
  
  Vector3d omg(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  Gyro packet(omg, msg->header.stamp.toSec());

  // dcm estimation 
  dcm_estimator_->update(packet);
 
  // write log 
  Matrix3d dcm_cov = dcm_estimator_->getCovariance();  
  Matrix3d dcm_R   = static_cast<DcmEstimator*>(dcm_estimator_)->getR();
  Vector3d dcm_rpy = dcm2rpy(dcm_R);
  writeLog(msg->header.stamp.toSec(), dcm_rpy, dcm_cov);

  // publish to rviz
  viz_.publishPose(msg->header.stamp.toSec(), dcm_R);
}

} // namespace att_est

using namespace att_est;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "att_est");

  ROS_INFO_STREAM("Starting att_est node");
  ros::NodeHandle nh;
  TestAttEstimation estimator(nh);
  while(ros::ok() && !estimator.quit())
  {
    ros::spinOnce();
    estimator.rate().sleep();
  }
  ROS_INFO_STREAM("Node killed.");
}

