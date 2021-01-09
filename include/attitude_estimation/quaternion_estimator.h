#ifndef _ATT_EST_QUATERNION_ESTIMATOR_H_
#define _ATT_EST_QUATERNION_ESTIMATOR_H_

#include "attitude_estimation/abstract_estimator.h"

namespace att_est
{

class QuaternionEstimator final : public AbstractEstimator
{
public:
  QuaternionEstimator(Vector3d gyro_noise_sq_continuous, double dt)
    : AbstractEstimator(gyro_noise_sq_continuous, dt)
  {
    quat_.setIdentity(); 
  }
  virtual ~QuaternionEstimator() {}

  inline Eigen::Quaterniond getQuat() const { return quat_; }

  virtual void propagateState(const Gyro& gyro) override
  {
    const double omega_norm = gyro.gyr_.norm(); 
    const double w = cos(omega_norm*dt_/2.0);
    Vector3d qv = (gyro.gyr_ / omega_norm) * sin(omega_norm*dt_/2.0);
    Eigen::Quaterniond dq(w, qv.x(), qv.y(), qv.z());
    quat_ = dq * quat_;
  }

  virtual void propagateCovariance(const Vector3d& phi) override
  {
    // TODO: need to derive
  }

private:
  Eigen::Quaterniond quat_;
}; // class QuaternionEstimator

} // namespace att_est

#endif // _ATT_EST_QUATERNION_ESTIMATOR_H_

