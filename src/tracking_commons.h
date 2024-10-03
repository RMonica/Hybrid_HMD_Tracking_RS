/*
 * Copyright 2022-2024 Riccardo Monica
 * 
 * This software is distributed under the 3-clause BSD license.
 * You should have received a copy of the 3-clause BSD license
 * along with this software. If not, see
 * <https://opensource.org/license/bsd-3-clause>
 */

#ifndef TRACKING_COMMONS_H
#define TRACKING_COMMONS_H

#ifndef M_PI
  #define M_PI 3.1415926535
#endif

#include <vector>
#include <stdint.h>
#include <iostream>

#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace Tracking
{
  typedef uint32_t uint32;
  typedef uint64_t uint64;

  template <typename T>
  inline T SQR(const T & t) {return t * t; }
  template <typename T>
  inline T CUBE(const T & t) {return t * t * t; }

  inline Eigen::Affine3f FloatvToAffine3f(const float * mat)
  {
    Eigen::Affine3f result;
    for (uint32 y = 0; y < 3; y++)
      for (uint32 x = 0; x < 4; x++)
      {
        const float v = mat[x + y * 4];
        if (x < 3)
          result.linear()(y, x) = v;
        else
          result.translation()(y) = v;
      }

    return result;
  }

  inline void Affine3fToFloatv(const Eigen::Affine3f & s, float * d)
  {
    for (uint32 y = 0; y < 4; y++)
      for (uint32 x = 0; x < 4; x++)
      {
        const float v = s.matrix()(y, x);
        d[y * 4 + x] = v;
      }
  }

  float DistanceInMeters(const Eigen::Affine3f & a, const Eigen::Affine3f & b);

  float DistanceInRadians(const Eigen::Affine3f & a, const Eigen::Affine3f & b);

  float DistanceInDegrees(const Eigen::Affine3f & a, const Eigen::Affine3f & b);

  Eigen::Quaternionf SafeSlerp(const Eigen::Quaternionf & Q1, const Eigen::Quaternionf & Q_2, const float ALPHA);

  Eigen::Matrix3f ApproxNormalizeRotation(const Eigen::Matrix3f & r);

  Eigen::Affine3f InterpolateAffine3f(const Eigen::Affine3f & m1, const Eigen::Affine3f & m2, const float alpha);
  Eigen::Affine3f PowAffine3f(const Eigen::Affine3f & m, const float exp);

  Eigen::Matrix3f InterpolateRotation(const Eigen::Matrix3f &m1, const Eigen::Matrix3f &m2, const float alpha);
  Eigen::Vector3f InterpolateTranslation(const Eigen::Vector3f &m1, const Eigen::Vector3f &m2, const float alpha);

  Eigen::Affine3f InterpolateAffine3fApproxComm(const Eigen::Affine3f & m1, const Eigen::Affine3f & m2,
                                                const float alpha1, const float alpha2);

  bool SolveQuadraticEquation(const float a, const float b, const float c, float & x1, float & x2);
}

#endif // TRACKING_COMMONS_H
