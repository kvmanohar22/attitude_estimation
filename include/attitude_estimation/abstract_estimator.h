#ifndef _ATT_EST_ABSTRACT_ESTIMATOR_H_
#define _ATT_EST_ABSTRACT_ESTIMATOR_H_

#include "attitude_estimation/global.h"
#include "attitude_estimation/gyro.h"

namespace att_est
{

class AbstractEstimator
{
public:
  AbstractEstimator(Vector3d gyro_noise_sq_continuous, double dt)
  {
    t_ = 0.0; 
    cov_.setZero();
    cov_gd_.setZero();

    cov_gd_(0,0) = gyro_noise_sq_continuous(0) / dt;
    cov_gd_(1,1) = gyro_noise_sq_continuous(1) / dt;
    cov_gd_(2,2) = gyro_noise_sq_continuous(2) / dt;
  }

  virtual ~AbstractEstimator() {}

  inline void setInitTime(const double t0) { t_ = t0; }
  inline Matrix3d getCovariance() const { return cov_; }

  /// propagates state
  virtual void propagateState(const Gyro& gyro) =0;

  /// propagates covariance (input is bias compensated omega
  virtual void propagateCovariance(const Vector3d& phi) =0;

  /// updates state and covariance
  virtual void update(const Gyro& gyro)
  {
    dt_ = gyro.t_ - t_; 
    propagateState(gyro);
    
    // we are not doing any bias compensation
    propagateCovariance(gyro.gyr_);
    
    // update time
    t_ = gyro.t_; 
  }

protected:
  double      t_;      //!< latest time 
  Matrix3d    cov_;    //!< covariance of state
  Matrix3d    cov_gd_; //!< (discrete) gyroscope noise covariance matrix
  double      dt_;     //!< latest delta t 
}; // class AbstractEstimator

} // namespace att_est

#endif // _ATT_EST_ABSTRACT_ESTIMATOR_H_

