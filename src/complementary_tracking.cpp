/*
 * Copyright 2022-2024 Riccardo Monica
 * 
 * This software is distributed under the 3-clause BSD license.
 * You should have received a copy of the 3-clause BSD license
 * along with this software. If not, see
 * <https://opensource.org/license/bsd-3-clause>
 */

#include "complementary_tracking.h"

namespace Tracking
{

ComplementaryTracking::ComplementaryTracking()
{
  SetParameter(PARAM_ALPHA, 0.003627f);
  Reset();
}

void ComplementaryTracking::Reset(const Eigen::Affine3f & initial_world_to_oculus,
                                  const Eigen::Affine3f & initial_oculus_origin_to_oculus)
{
  prev_world_to_motive = initial_world_to_oculus;
  prev_oculus_origin_to_oculus = initial_oculus_origin_to_oculus;

  oculus_origin_mat = initial_world_to_oculus * initial_oculus_origin_to_oculus.inverse();
  oculus_mat = initial_world_to_oculus;

  position_state = STATE_INIT;
  rotation_state = STATE_INIT;

  has_converged = false;
}

void ComplementaryTracking::Reset()
{
  prev_world_to_motive.linear() = Eigen::Matrix3f::Zero();
  prev_oculus_origin_to_oculus.linear() = Eigen::Matrix3f::Zero();
  position_state = STATE_INIT;
  rotation_state = STATE_INIT;

  has_converged = false;
}

void ComplementaryTracking::Update(const Eigen::Affine3f & m_world_to_motive_mat,
                                   const Eigen::Affine3f & m_oculus_origin_to_oculus)
{
  if (prev_world_to_motive.linear() == Eigen::Matrix3f::Zero())
  {
    prev_world_to_motive = m_world_to_motive_mat;
    prev_oculus_origin_to_oculus = m_oculus_origin_to_oculus;
    oculus_origin_mat = Eigen::Affine3f::Identity();
    oculus_mat = Eigen::Affine3f::Identity();
    position_state = STATE_INIT;
    rotation_state = STATE_INIT;
    return;
  }

  is_motive_lost = (m_world_to_motive_mat.linear() == prev_world_to_motive.linear()) &&
                   (m_world_to_motive_mat.translation() == prev_world_to_motive.translation());

  if (!is_motive_lost)
  {
    Eigen::Affine3f m_world_to_oculus = oculus_origin_mat * m_oculus_origin_to_oculus;

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
      const float alpha = m_params[PARAM_ALPHA];

      //const Eigen::Matrix4f M1 = m_world_to_motive_mat.matrix();
      //const Eigen::Matrix4f M2 = m_world_to_oculus.matrix();

      //new_oculus.matrix() = M1.pow(alpha) * M2.pow(1.0f - alpha);
      //new_oculus.matrix() = ((M1 * M2.inverse()).log() * alpha).exp() * M2;
      new_oculus = InterpolateAffine3f(m_world_to_oculus, m_world_to_motive_mat, alpha);

      position_state = STATE_SMALL_DIFF;
      rotation_state = STATE_SMALL_DIFF;
    }

    new_oculus.linear() = ApproxNormalizeRotation(new_oculus.linear());

    oculus_mat = new_oculus;
    oculus_origin_mat = oculus_mat * m_oculus_origin_to_oculus.inverse();
  }

  oculus_mat = oculus_origin_mat * m_oculus_origin_to_oculus;

  prev_world_to_motive = m_world_to_motive_mat;
  prev_oculus_origin_to_oculus = m_oculus_origin_to_oculus;
}

}
