#ifndef _ATT_EST_UTILS_H_
#define _ATT_EST_UTILS_H_

#include "attitude_estimation/global.h"
#include <math.h>

namespace att_est
{

// skew symmetric matrix representation of a vector
inline Matrix3d sqew(const Vector3d& v)
{
  Matrix3d m;
  m <<      0, -v.z(),  v.y(),
        v.z(),      0, -v.x(),
       -v.y(),  v.x(),      0;
  return m;
}

/// Converts Direction cosine matrix to roll, pitch and yaw
inline Vector3d dcm2rpy(const Matrix3d& R)
{
  Vector3d rpy;
  rpy[1] = atan2( -R(2,0), sqrt( pow( R(0,0), 2 ) + pow( R(1,0), 2 ) ) );
  if( fabs( rpy[1] - M_PI/2 ) < 0.00001 )
  {
    rpy[2] = 0;
    rpy[0] = -atan2( R(0,1), R(1,1) );
  }
  else
  {
    if( fabs( rpy[1] + M_PI/2 ) < 0.00001 )
    {
      rpy[2] = 0;
      rpy[0] = -atan2( R(0,1), R(1,1) );
    }
    else
    {
      rpy[2] = atan2( R(1,0)/cos(rpy[1]), R(0,0)/cos(rpy[1]) );
      rpy[0] = atan2( R(2,1)/cos(rpy[1]), R(2,2)/cos(rpy[1]) );
    }
  }
  return rpy;
}

// roll angle
inline Matrix3d rx(const double r)
{
  const double cos_r = cos(r);
  const double sin_r = sin(r);
  Matrix3d R;
  R << 1.0,    0.0,   0.0,
       0.0,  cos_r, sin_r,
       0.0, -sin_r, cos_r;
  return R;
}

// pitch angle
inline Matrix3d ry(const double p)
{
  const double cos_p = cos(p);
  const double sin_p = sin(p);
  Matrix3d R;
  R <<  cos_p, 0.0, -sin_p,
          0.0, 1.0,    0.0,
        sin_p, 0.0,  cos_p;
  return R;
}

// yaw angle
inline Matrix3d rz(const double y)
{
  const double cos_y = cos(y);
  const double sin_y = sin(y);
  Matrix3d R;
  R << cos_y, sin_y, 0.0,
      -sin_y, cos_y, 0.0,
         0.0,   0.0, 1.0;
  return R;
}

} // namespace att_est

#endif // _ATT_EST_UTILS_H_

