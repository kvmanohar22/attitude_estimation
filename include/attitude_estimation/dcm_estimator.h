#ifndef _ATT_EST_DCM_ESTIMATOR_H_
#define _ATT_EST_DCM_ESTIMATOR_H_

#include "attitude_estimation/abstract_estimator.h"
#include "attitude_estimation/utils.h"
#include "attitude_estimation/gyro.h"
#include <unsupported/Eigen/src/MatrixFunctions/MatrixExponential.h>

namespace att_est
{

class DcmEstimator final : public AbstractEstimator
{
public:
  DcmEstimator(Vector3d gyro_noise_continuous, double dt) : AbstractEstimator()
  {
    dR_ij_.setIdentity();
    R_w_b_.setIdentity();
    cov_gd_.setZero();

    // convert from continuous to discrete white noise
    cov_gd_(0,0) = gyro_noise_continuous(0) / dt;
    cov_gd_(1,1) = gyro_noise_continuous(1) / dt;
    cov_gd_(2,2) = gyro_noise_continuous(2) / dt;
  }
  virtual ~DcmEstimator() {}

  inline Matrix3d getR() const { return R_w_b_; }

  /// computes the right jacobian of SO(3) group
  Matrix3d computeSO3rightJac(const Vector3d& phi)
  {
    Matrix3d jac;
    const double phi_norm = phi.norm();
    const double phi_norm2 = phi_norm * phi_norm;
    const Matrix3d phi_hat = sqew(phi);
    jac = Matrix3d::Identity()
            - ((1-cos(phi_norm))/phi_norm2) * phi_hat
            + ((phi_norm - sin(phi_norm))/(phi_norm2*phi_norm)) * phi_hat * phi_hat;
    return jac;
  }

  virtual void propagateState(const Gyro& gyro) override
  {
    // compute matrix exponential
    Matrix3d sq = sqew(gyro.gyr_); 
    dR_jminus_j_ = sq.exp(); 
    dR_ij_ = dR_ij_ * dR_jminus_j_;

    // since initial attitude is identity
    R_w_b_ = dR_ij_;
  }

  virtual void propagateCovariance(const Vector3d& phi) override
  {
    Matrix3d r_jac = computeSO3rightJac(phi);
    cov_ = dR_jminus_j_.transpose() * cov_ * dR_jminus_j_ + r_jac * cov_gd_ * r_jac.transpose() * (dt_*dt_); 
  }

private:
  Matrix3d    dR_jminus_j_;   //!< latest imu's delta rotation measurement 
  Matrix3d    cov_gd_;        //!< discrete covariance matrix of gyroscope 
  Matrix3d    R_w_b_;         //!< transforms vectors from (b)ody frame to (w)orld frame
  Matrix3d    dR_ij_;         //!< preintegrated rotation delta measurement
}; // class DcmEstimator

} // namespace att_est

#endif // _ATT_EST_DCM_ESTIMATOR_H_

