/*
 * Copyright 2022-2024 Riccardo Monica
 * 
 * This software is distributed under the 3-clause BSD license.
 * You should have received a copy of the 3-clause BSD license
 * along with this software. If not, see
 * <https://opensource.org/license/bsd-3-clause>
 */

#include "directed_complementary_tracking.h"

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

DirectedComplementaryTracking::DirectedComplementaryTracking()
{
  SetParameter(PARAM_ROTATION_LIMIT, 0.019630f);
  SetParameter(PARAM_TRANSLATION_LIMIT, 0.000130f);

  SetParameter(PARAM_ROTATION_DYN_LIMIT, 0.036499f);
  SetParameter(PARAM_ROTATION_CURV_LIMIT, 1.579485f); // degrees / meter
  SetParameter(PARAM_TRANSLATION_DYN_LIMIT, 0.092148f);

  SetParameter(PARAM_ROTATION_DYN2_LIMIT, 0.032735f);
  SetParameter(PARAM_ROTATION_CURV2_LIMIT, 0.605983f); // degrees / meter
  SetParameter(PARAM_TRANSLATION_DYN2_LIMIT, 0.073467f);

  SetParameter(PARAM_MULT, 1.0f);

  Reset();
}

bool DirectedComplementaryTracking::SetParameter(const int parameter_id, const float value)
{
  switch (parameter_id)
  {
    case PARAM_ROTATION_LIMIT:
    case PARAM_ROTATION_DYN_LIMIT:
    case PARAM_ROTATION_CURV_LIMIT:
    case PARAM_TRANSLATION_DYN_LIMIT:
    case PARAM_TRANSLATION_LIMIT:
    case PARAM_ROTATION_DYN2_LIMIT:
    case PARAM_ROTATION_CURV2_LIMIT:
    case PARAM_TRANSLATION_DYN2_LIMIT:
    case PARAM_MULT:
      return ITracking::SetParameter(parameter_id, value);
    default:
      return false;
  }
}

void DirectedComplementaryTracking::Reset(const Eigen::Affine3f & initial_world_to_oculus,
                                          const Eigen::Affine3f & initial_oculus_origin_to_oculus)
{
  prev_world_to_motive = initial_world_to_oculus;
  prev_oculus_origin_to_oculus = initial_oculus_origin_to_oculus;
  oculus_mat = initial_world_to_oculus;

  position_state = STATE_INIT;
  rotation_state = STATE_INIT;

  has_converged = false;
}

void DirectedComplementaryTracking::Reset()
{
  prev_world_to_motive.linear() = Eigen::Matrix3f::Zero();
  prev_oculus_origin_to_oculus.linear() = Eigen::Matrix3f::Zero();

  position_state = STATE_INIT;
  rotation_state = STATE_INIT;

  has_converged = false;
}

static void GetSwingTwistDecomposition(const Eigen::Matrix3f & original_rotation,
                                       const Eigen::Vector3f & axis,
                                       Eigen::AngleAxisf & twist,
                                       Eigen::AngleAxisf & swing)
{
  const Eigen::Quaternionf s(original_rotation);
  const float u = (s.x() * axis[0] + s.y() * axis[1] + s.z() * axis[2]);
  if (std::abs(u) < 0.0001f)
  {
    twist = Eigen::AngleAxisf(s);
    swing = Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitX());
    return;
  }

  // https://github.com/cuauv/software/blob/master/fishbowl/geometry.cpp#L73
  Eigen::Vector3f p = axis * u;
  Eigen::Quaternionf qtwist = Eigen::Quaternionf(s.w(), p[0], p[1], p[2]);
  qtwist.normalize();
  Eigen::Quaternionf qswing = s * qtwist.inverse();

  twist = Eigen::AngleAxisf(qtwist);
  swing = Eigen::AngleAxisf(qswing);
}

static void GetTwistSwingDecomposition(const Eigen::Matrix3f & original_rotation,
                                       const Eigen::Vector3f & axis,
                                       Eigen::AngleAxisf & swing,
                                       Eigen::AngleAxisf & twist)
{
  GetSwingTwistDecomposition(original_rotation.transpose(), axis, twist, swing);
  twist.angle() = -twist.angle();
  swing.angle() = -swing.angle();
}

void DirectedComplementaryTracking::Update(const Eigen::Affine3f & m_world_to_motive_mat,
                                           const Eigen::Affine3f & m_oculus_origin_to_oculus)
{
  if (prev_world_to_motive.linear() == Eigen::Matrix3f::Zero())
  {
    prev_world_to_motive = m_world_to_motive_mat;
    prev_oculus_origin_to_oculus = m_oculus_origin_to_oculus;
    oculus_mat = Eigen::Affine3f::Identity();
    position_state = STATE_INIT;
    rotation_state = STATE_INIT;
    return;
  }

  //const Eigen::Affine3f dt = oculus_mat.inverse() * prev_world_to_motive;
  //oculus_mat = oculus_mat * dt * prev_oculus_origin_to_oculus.inverse() * m_oculus_origin_to_oculus * dt.inverse();
  oculus_mat = oculus_mat * prev_oculus_origin_to_oculus.inverse() * m_oculus_origin_to_oculus;

  is_motive_lost = (m_world_to_motive_mat.linear() == prev_world_to_motive.linear()) &&
                   (m_world_to_motive_mat.translation() == prev_world_to_motive.translation());

  if (!is_motive_lost)
  {
    Eigen::Affine3f m_world_to_oculus = oculus_mat;

    float distance = DistanceInMeters(m_world_to_motive_mat, m_world_to_oculus);
    float angle = DistanceInDegrees(m_world_to_motive_mat, m_world_to_oculus);

    Eigen::Affine3f new_oculus;

    const float large_angle_threshold = 30.0f;
    const float large_distance_threshold = 0.5f;
    if (distance > large_distance_threshold || angle > large_angle_threshold)
    {
      new_oculus = m_world_to_motive_mat;

      if (angle > large_angle_threshold)
        rotation_state = STATE_REPOSITION;
      if (distance > large_distance_threshold)
        position_state = STATE_REPOSITION;
    }
    else
    {
      float oculus_dist_diff = DistanceInMeters(m_oculus_origin_to_oculus, prev_oculus_origin_to_oculus);
      float oculus_angle_diff = DistanceInDegrees(m_oculus_origin_to_oculus, prev_oculus_origin_to_oculus);

      const float mult = m_params[PARAM_MULT];
      const float rotation_tot_limit = std::max(m_params[PARAM_ROTATION_LIMIT],
                                       std::max(m_params[PARAM_ROTATION_DYN_LIMIT] * mult * oculus_angle_diff,
                                                m_params[PARAM_ROTATION_CURV_LIMIT] * mult * oculus_dist_diff));
      const float translation_tot_limit = std::max(m_params[PARAM_TRANSLATION_LIMIT],
                                                   m_params[PARAM_TRANSLATION_DYN_LIMIT] * mult * oculus_dist_diff);

      float frameNumbers = (distance / translation_tot_limit + angle / rotation_tot_limit);
      if (false)
      {
        const float P = 0.07;
        const float rotation_tot_limit_P = rotation_tot_limit / P;
        const float translation_tot_limit_P = translation_tot_limit / P;

        const float a = P;
        const float b = -(distance / translation_tot_limit_P + angle / rotation_tot_limit_P);
        const float c = (distance * angle) / (translation_tot_limit_P * rotation_tot_limit_P);

        float n1;
        float n2;
        if (SolveQuadraticEquation(a, b, c, n1, n2))
        {
          frameNumbers = n1;
        }
      }

      if (frameNumbers > 1.0f && !has_converged)
      {
          float ALPHA = 1 / frameNumbers;

          new_oculus = InterpolateAffine3f(m_world_to_oculus, m_world_to_motive_mat, ALPHA);

          position_state = STATE_MEDIUM_DIFF;
          rotation_state = STATE_MEDIUM_DIFF;
      }
      else
      {
          new_oculus = m_world_to_motive_mat;
          position_state = STATE_SMALL_DIFF;
          rotation_state = STATE_SMALL_DIFF;

          has_converged = true;
      }

      const float translation_dyn_limit = m_params[PARAM_TRANSLATION_DYN2_LIMIT] * mult;
      const float rotation_dyn_limit = m_params[PARAM_ROTATION_DYN2_LIMIT] * mult;
      const float rotation_curv_limit = m_params[PARAM_ROTATION_CURV2_LIMIT] * mult / 180.0f * float(M_PI);

      {
        const Eigen::Affine3f oculus_oculus_diff = prev_oculus_origin_to_oculus.inverse() * m_oculus_origin_to_oculus;
        const Eigen::Affine3f oculus_motive_diff = new_oculus.inverse() * m_world_to_motive_mat;

        const Eigen::Vector3f translation_constrain_axis = (oculus_oculus_diff.translation().squaredNorm() > SQR(0.0001f)) ?
              oculus_oculus_diff.translation().normalized() : Eigen::Vector3f::UnitX();
        const Eigen::Vector3f full_translation = oculus_motive_diff.translation();
        const Eigen::Vector3f req_translation = full_translation.dot(translation_constrain_axis) * translation_constrain_axis;

        Eigen::Vector3f act_translation = req_translation;
        {
          const float limit = oculus_oculus_diff.translation().norm() * translation_dyn_limit;
          if (act_translation.norm() > limit + 0.000001f)
            act_translation = limit * act_translation / act_translation.norm();
        }

        const Eigen::AngleAxisf oculus_oculus_diff_aa(oculus_oculus_diff.linear());
        const Eigen::Vector3f rotation_constrain_axis = oculus_oculus_diff_aa.axis();
        Eigen::AngleAxisf useless_rotation_swing, rotation_twist;
        GetSwingTwistDecomposition(oculus_motive_diff.linear(), rotation_constrain_axis, rotation_twist, useless_rotation_swing);

        float act_rotation_angle;
        {
          const float limit = std::abs(oculus_oculus_diff_aa.angle()) * rotation_dyn_limit;
          if (std::abs(rotation_twist.angle()) < limit + 0.00001f)
            act_rotation_angle = rotation_twist.angle();
          else
            act_rotation_angle = limit * rotation_twist.angle() / std::abs(rotation_twist.angle());
        }

        const Eigen::Matrix3f act_rotation = Eigen::AngleAxisf(act_rotation_angle, rotation_twist.axis()).matrix();

        const Eigen::Vector3f curvature_constrain_perp_axis = translation_constrain_axis;
        Eigen::AngleAxisf curvature_swing, useless_curvature_twist;
        GetTwistSwingDecomposition(oculus_motive_diff.linear(), curvature_constrain_perp_axis, curvature_swing, useless_curvature_twist);

        Eigen::AngleAxisf act_curvature = curvature_swing;
        {
          const float limit = std::abs(oculus_oculus_diff.translation().norm()) * rotation_curv_limit;
          if (std::abs(curvature_swing.angle()) > limit + 0.00001f)
            act_curvature.angle() = limit * curvature_swing.angle() / std::abs(curvature_swing.angle());
        }

        Eigen::Affine3f correction = Eigen::Affine3f::Identity();
        {
          const Eigen::Affine3f corr_curv(act_curvature);
          const Eigen::Affine3f corr_rot(act_rotation);
          Eigen::Affine3f corr_transl = Eigen::Affine3f::Identity();
          corr_transl.translation() = act_translation;
          correction = SE3ToEigen((EigenToSE3(corr_curv).log() + EigenToSE3(corr_rot).log() + EigenToSE3(corr_transl).log()).exp());
        }
        new_oculus = new_oculus * correction;
      }
    }

    oculus_mat = new_oculus;
  }

  oculus_mat.linear() = ApproxNormalizeRotation(oculus_mat.linear());

  prev_world_to_motive = m_world_to_motive_mat;
  prev_oculus_origin_to_oculus = m_oculus_origin_to_oculus;
}

}
