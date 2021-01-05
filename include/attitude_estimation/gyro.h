#ifndef _ATT_EST_GYRO_H_
#define _ATT_EST_GYRO_H_

#include "attitude_estimation/global.h"

namespace att_est
{

/// Gyroscope data holder
class Gyro
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Gyro()
  {
    gyr_.setZero();
    t_ = 0.0;
  }

  Gyro(const Vector3d& gyr, const double t)
    : gyr_(gyr),
      t_(t)
  {}
  virtual ~Gyro() {}

private:
  Vector3d gyr_; //!< gyroscope data
  double   t_;   //!< timestamp
}; // class Gyro

} // namespace att_est

#endif // _ATT_EST_GYRO_H_

