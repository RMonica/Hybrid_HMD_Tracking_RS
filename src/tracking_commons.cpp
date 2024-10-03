/*
 * Copyright 2022-2024 Riccardo Monica
 * 
 * This software is distributed under the 3-clause BSD license.
 * You should have received a copy of the 3-clause BSD license
 * along with this software. If not, see
 * <https://opensource.org/license/bsd-3-clause>
 */

#include "tracking_commons.h"

//#include <unsupported/Eigen/MatrixFunctions>

#include <manif/manif.h>

typedef double Scalar;
using Vector6 = Eigen::Matrix<Scalar, 6, 1>;
using Matrix6 = Eigen::Matrix<Scalar, 6, 6>;
using SE3 = manif::SE3d;
using SE3Tangent = manif::SE3Tangentd;

static SE3 EigenToSE3(const Eigen::Affine3f & m)
{
  Eigen::Matrix4d matr = m.cast<double>().matrix();
  Eigen::Isometry3d transfEigen;
  transfEigen = matr;
  return transfEigen;
}

static Eigen::Affine3f SE3ToEigen(const SE3 & se3)
{
  Eigen::Affine3f result;
  result.matrix() = se3.isometry().matrix().cast<float>();
  return result;
}

namespace Tracking
{

Eigen::Quaternionf SafeSlerp(const Eigen::Quaternionf & Q1, const Eigen::Quaternionf & Q_2, const float ALPHA)
{
  Eigen::Quaternionf Q2 = Q_2;
  if (Q1.x() * Q2.x() + Q1.y() * Q2.y() + Q1.z() * Q2.z() + Q1.w() * Q2.w() < 0.0f)
  {
    Q2.x() = -Q2.x();
    Q2.y() = -Q2.y();
    Q2.z() = -Q2.z();
    Q2.w() = -Q2.w();
  }
  return Q1.slerp(ALPHA, Q2).normalized();
}

Eigen::Matrix3f ApproxNormalizeRotation(const Eigen::Matrix3f & r)
{
  Eigen::Matrix3f result;
  result.col(0) = r.col(0).normalized();
  result.col(1) = r.col(1).normalized();
  result.col(2) = result.col(0).cross(result.col(1));
  return result;
}

Eigen::Affine3f InterpolateAffine3f(const Eigen::Affine3f & m1, const Eigen::Affine3f & m2, const float alpha)
{
  return m1 * SE3ToEigen((EigenToSE3(m1.inverse() * m2).log() * alpha).exp());
}

Eigen::Affine3f PowAffine3f(const Eigen::Affine3f & m, const float exp)
{
  return SE3ToEigen((EigenToSE3(m).log() * exp).exp());
}

Eigen::Matrix3f InterpolateRotation(const Eigen::Matrix3f & m1, const Eigen::Matrix3f & m2, const float alpha)
{
  const Eigen::Matrix3f diff = m1.transpose() * m2;
  const Eigen::AngleAxisf aa(diff);
  const float angle = aa.angle() * alpha;
  const Eigen::Vector3f axis = aa.axis();
  const Eigen::Matrix3f result = m1 * Eigen::AngleAxisf(angle, axis).matrix();
  return result;
}

Eigen::Vector3f InterpolateTranslation(const Eigen::Vector3f &m1, const Eigen::Vector3f &m2, const float alpha)
{
  return m1 * (1.0f - alpha) + m2 * alpha;
}

Eigen::Affine3f InterpolateAffine3fApproxComm(const Eigen::Affine3f & m1, const Eigen::Affine3f & m2,
                                              const float alpha1, const float alpha2)
{
  Eigen::Quaternionf r1(m1.linear());
  Eigen::Quaternionf r2(m2.linear());
  Eigen::Vector4f rv1(r1.x(), r1.y(), r1.z(), r1.w());
  Eigen::Vector4f rv2(r2.x(), r2.y(), r2.z(), r2.w());
  Eigen::Quaternionf rr((rv1 * alpha1 + rv2 * alpha2).normalized());

  Eigen::Vector3f t1 = m1.translation();
  Eigen::Vector3f t2 = m2.translation();
  Eigen::Vector3f tr = t1 * alpha1 + t2 * alpha2;

  Eigen::Affine3f result;
  result.linear() = rr.matrix();
  result.translation() = tr;

  return result;
}

float DistanceInMeters(const Eigen::Affine3f & a, const Eigen::Affine3f & b)
{
  return (b.inverse() * a).translation().norm();
}

float DistanceInRadians(const Eigen::Affine3f & a, const Eigen::Affine3f & b)
{
  return std::abs(Eigen::AngleAxisf((b.inverse() * a).linear()).angle());
}

float DistanceInDegrees(const Eigen::Affine3f & a, const Eigen::Affine3f & b)
{
  return DistanceInRadians(a, b) * 180.0f / float(M_PI);
}

bool SolveQuadraticEquation(const float a, const float b, const float c, float & x1, float & x2)
{
  const float delta = b * b - 4 * a * c;
  if (delta < 0.0f)
    return false;

  x1 = (-b + std::sqrt(delta)) / (2 * a);
  x2 = (-b - std::sqrt(delta)) / (2 * a);
  return true;
}

}
