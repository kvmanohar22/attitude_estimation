#ifndef _ATT_EST_EULER_ESTIMATOR_H_
#define _ATT_EST_EULER_ESTIMATOR_H_

#include "attitude_estimation/global.h"
#include "attitude_estimation/abstract_estimator.h"

namespace att_est
{

class EulerEstimator final : public AbstractEstimator
{
public:
  EulerEstimator(Vector3d gyro_noise_sq_continuous, double dt)
    : AbstractEstimator(gyro_noise_sq_continuous, dt)
  {
    euler_angles_.setZero(); 
  }
  virtual ~EulerEstimator() {}

  Matrix3d getConjuateEulerRateInv(const Vector3d& theta)
  {
    const double sin_alpha = sin(theta(0)); 
    const double cos_alpha = cos(theta(0)); 
    const double cos_beta  = cos(theta(1)); 
    const double tan_beta  = tan(theta(1)); 

    Matrix3d E;
    E(0,0) = 1.0; 
    E(1,0) = 0.0; 
    E(2,0) = 0.0; 

    E(0,1) = sin_alpha * tan_beta;
    E(1,1) = cos_alpha;
    E(2,1) = sin_alpha / cos_beta;

    E(0,2) = cos_alpha * tan_beta;
    E(1,2) = -sin_alpha;
    E(2,2) = cos_alpha / cos_beta;

    return E;
  }

  inline Vector3d getEulerAngles() const { return euler_angles_; }

  /// Jacobian of conjugate euler rate matrix
  Matrix3d getConjugateEulerRateInvJac(const Vector3d& theta)
  {
    Matrix3d jac;
    // TODO

    return jac;
  }

  virtual void propagateState(const Gyro& gyro) override
  {
    euler_angles_ = euler_angles_ + getConjuateEulerRateInv(euler_angles_)*gyro.gyr_*dt_;
  }

  virtual void propagateCovariance(const Vector3d& phi) override
  {
    // TODO 
    return;
    
    /* 
    Matrix3d jac = getConjugateEulerRateInvJac(euler_angles_);
    Matrix3d E_theta = getConjuateEulerRateInv(euler_angles_);
    Matrix3d Ai = Matrix3d::Identity() + jac * phi * dt_;
    cov_ = Ai * cov_ * Ai.transpose() + E_theta * cov_gd_ * E_theta.transpose() * dt_*dt_; 
    */ 
  }

private:
  Vector3d   euler_angles_; 
}; // class EulerEstimator

} // namespace att_est

#endif // _ATT_EST_EULER_ESTIMATOR_H_

