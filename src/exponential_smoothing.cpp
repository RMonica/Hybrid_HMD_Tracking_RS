/*
 * Copyright 2022-2024 Riccardo Monica
 * 
 * This software is distributed under the 3-clause BSD license.
 * You should have received a copy of the 3-clause BSD license
 * along with this software. If not, see
 * <https://opensource.org/license/bsd-3-clause>
 */

#include "exponential_smoothing.h"

namespace Tracking
{

ExponentialSmoothing::ExponentialSmoothing()
{
  SetParameter(PARAM_ALPHA, 0.01f);
  Reset();
}

void ExponentialSmoothing::Reset(const Eigen::Affine3f & initial_world_to_oculus,
                                 const Eigen::Affine3f & initial_oculus_origin_to_oculus)
{
  prev_world_to_motive = initial_world_to_oculus;
  prev_oculus_mat = initial_world_to_oculus;

  oculus_origin_mat = initial_world_to_oculus * initial_oculus_origin_to_oculus.inverse();
  prev_oculus_origin_mat = oculus_origin_mat;
  oculus_mat = initial_oculus_origin_to_oculus;

  position_state = STATE_INIT;
  rotation_state = STATE_INIT;
}

void ExponentialSmoothing::Update(const Eigen::Affine3f & m_world_to_motive_mat,
                                   const Eigen::Affine3f & m_oculus_origin_to_oculus)
{
  if (prev_world_to_motive.linear() == Eigen::Matrix3f::Zero())
  {
    prev_world_to_motive = Eigen::Affine3f::Identity();
    prev_oculus_mat = Eigen::Affine3f::Identity();
    oculus_origin_mat = Eigen::Affine3f::Identity();
    prev_oculus_origin_mat = Eigen::Affine3f::Identity();
    oculus_mat = Eigen::Affine3f::Identity();
    position_state = STATE_INIT;
    rotation_state = STATE_INIT;
    return;
  }

  is_motive_lost = (m_world_to_motive_mat.linear() == prev_world_to_motive.linear()) &&
                   (m_world_to_motive_mat.translation() == prev_world_to_motive.translation());

  oculus_mat = oculus_origin_mat * m_oculus_origin_to_oculus;
  prev_oculus_mat = prev_oculus_origin_mat * m_oculus_origin_to_oculus;

  if (!is_motive_lost)
  {
    Eigen::Affine3f m_world_to_oculus = oculus_origin_mat * m_oculus_origin_to_oculus;

    float distance = DistanceInMeters(m_world_to_motive_mat, m_world_to_oculus);
    float angle = DistanceInDegrees(m_world_to_motive_mat, m_world_to_oculus);

    const float large_angle_threshold = 30.0f;
    const float large_distance_threshold = 0.5f;
    if (distance > large_distance_threshold || angle > large_angle_threshold)
    {
      oculus_mat = m_world_to_motive_mat;
      prev_oculus_mat = m_world_to_motive_mat;

      if (angle > large_angle_threshold)
        rotation_state = STATE_REPOSITION;
      if (distance > large_distance_threshold)
        position_state = STATE_REPOSITION;
    }
    else
    {
      oculus_mat = m_world_to_motive_mat;

      position_state = STATE_SMALL_DIFF;
      rotation_state = STATE_SMALL_DIFF;
    }
  }

  const float alpha = m_params[PARAM_ALPHA];
  const float tau = 0.0f; // predicting right now

  prev_oculus_mat = InterpolateAffine3fApproxComm(prev_oculus_mat, oculus_mat, 1.0f - alpha, alpha);
  oculus_mat = prev_oculus_mat;

  oculus_mat.linear() = ApproxNormalizeRotation(oculus_mat.linear());

  oculus_origin_mat = oculus_mat * m_oculus_origin_to_oculus.inverse();
  prev_oculus_origin_mat = prev_oculus_mat * m_oculus_origin_to_oculus.inverse();

  prev_world_to_motive = m_world_to_motive_mat;
}

}
