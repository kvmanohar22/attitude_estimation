#include "attitude_estimation/dcm_estimator.h"
#include "attitude_estimation/euler_estimator.h"
#include "attitude_estimation/quaternion_estimator.h"
#include "attitude_estimation/gyro.h"
#include "attitude_estimation/utils.h"
#include "attitude_estimation/visualizer.h"

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
  AbstractEstimator* dcm_estimator_; //<! DCM
  AbstractEstimator* elr_estimator_; //<! euler
  AbstractEstimator* qat_estimator_; //<! quaternions
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
  elr_estimator_ = new EulerEstimator(gyro_noise, dt);
  qat_estimator_ = new QuaternionEstimator(gyro_noise, dt);

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
  delete dcm_estimator_;
  delete elr_estimator_;
  delete qat_estimator_;
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
  elr_estimator_->update(packet);
  qat_estimator_->update(packet);
 
  // visualize
  Matrix3d dcm_cov = dcm_estimator_->getCovariance();  
  Matrix3d dcm_R   = static_cast<DcmEstimator*>(dcm_estimator_)->getR();

  Vector3d euler_angles = static_cast<EulerEstimator*>(elr_estimator_)->getEulerAngles();
  Eigen::Quaterniond quat = static_cast<QuaternionEstimator*>(qat_estimator_)->getQuat();

  // publish to rviz
  viz_.publishPoseAll(
      msg->header.stamp.toSec(),
      dcm_R, dcm_cov,
      euler_angles, dcm_cov,
      quat);
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

