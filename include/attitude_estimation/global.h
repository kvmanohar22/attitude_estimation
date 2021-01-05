#ifndef _ATT_EST_GLOBAL_H_
#define _ATT_EST_GLOBAL_H_

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Dense>

#include <cassert>
#include <string>
#include <vector>
#include <list>
#include <iostream>
#include <chrono>

#include <ros/ros.h>

namespace att_est
{
  static constexpr double PI = 3.1415926535;  //!< pi

  typedef Eigen::Matrix<double, 3, 3> Matrix3d;
  typedef Eigen::Matrix<double, 2, 2> Matrix2d;
  typedef Eigen::Matrix<double, 3, 1> Vector3d;
  typedef Eigen::Matrix<double, 2, 1> Vector2d;

} // namespace att_est

#endif
