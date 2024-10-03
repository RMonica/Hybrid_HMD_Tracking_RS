/*
 * Copyright 2022-2024 Riccardo Monica
 * 
 * This software is distributed under the 3-clause BSD license.
 * You should have received a copy of the 3-clause BSD license
 * along with this software. If not, see
 * <https://opensource.org/license/bsd-3-clause>
 */

#include "oculus_original_tracking.h"

namespace Tracking
{
  bool OculusOriginTracking::SetParameter(const int parameter_id, const float value)
  {
    switch (parameter_id)
    {
      case PARAM_ROTATION_LIMIT:
      case PARAM_ROTATION_DYN_LIMIT:
      case PARAM_TRANSLATION_DYN_LIMIT:
      case PARAM_TRANSLATION_LIMIT:
        return ITracking::SetParameter(parameter_id, value);
      default:
        return false;
    }
  }

  OculusOriginTracking::OculusOriginTracking()
  {
    SetParameter(PARAM_ROTATION_LIMIT, 0.01f);
    SetParameter(PARAM_ROTATION_DYN_LIMIT, 0.0f);
    SetParameter(PARAM_TRANSLATION_DYN_LIMIT, 0.01f);
    SetParameter(PARAM_TRANSLATION_LIMIT, 0.0025f);

    Reset();
  }

  void OculusOriginTracking::Update(const Eigen::Affine3f & m_world_to_motive_mat,
                                    const Eigen::Affine3f & m_oculus_origin_to_oculus)
  {
    Eigen::Affine3f m_motive_world_to_oculus_origin = m_world_to_motive_mat * m_oculus_origin_to_oculus.inverse();

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
      float angle = DistanceInDegrees(m_motive_world_to_oculus_origin, oculus_origin_mat);
      float distance = DistanceInMeters(m_motive_world_to_oculus_origin, oculus_origin_mat);

      Eigen::Affine3f new_oculus_origin;

      const float large_angle_threshold = 30.0f;
      const float large_distance_threshold = 0.5f;
      if (distance > large_distance_threshold || angle > large_angle_threshold)
      {
        new_oculus_origin = m_motive_world_to_oculus_origin;

        if (angle > large_angle_threshold)
          rotation_state = STATE_REPOSITION;
        if (distance > large_distance_threshold)
          position_state = STATE_REPOSITION;
      }
      else
      {
        float oculus_dist_diff = DistanceInMeters(m_oculus_origin_to_oculus, prev_oculus_origin_to_oculus);
        float oculus_angle_diff = DistanceInDegrees(m_oculus_origin_to_oculus, prev_oculus_origin_to_oculus);

        //float maxDistance = 0.00025f + 0.001f * distanceAlfa;
        //float maxAngle = 0.0002f + 0.00f * angleAlfa;
        const float maxDistance = m_params[PARAM_TRANSLATION_LIMIT] + m_params[PARAM_TRANSLATION_DYN_LIMIT] * oculus_dist_diff;
        const float maxAngle = m_params[PARAM_ROTATION_LIMIT] + m_params[PARAM_ROTATION_DYN_LIMIT] * oculus_angle_diff;

        if (distance > maxDistance || angle > maxAngle)
        {
          float frameNumbers = std::max(angle / maxAngle,
                                        distance / maxDistance);
          float ALPHA_DISTANCE = 1 / frameNumbers;

          /*
          new_oculus_origin.matrix() = m_motive_world_to_oculus_origin.pow(ALPHA_DISTANCE) *
              oculus_origin_mat.pow(1.0f - ALPHA_DISTANCE);
          */

          Eigen::Vector3f T1 = m_motive_world_to_oculus_origin.translation();
          Eigen::Vector3f T2 = oculus_origin_mat.translation();
          Eigen::Vector3f Tm = T1 * ALPHA_DISTANCE + T2 * (1.0f - ALPHA_DISTANCE);

          new_oculus_origin.translation() = Tm;

          Eigen::Quaternionf Q1(m_motive_world_to_oculus_origin.linear());
          Eigen::Quaternionf Q2(oculus_origin_mat.linear());
          Eigen::Quaternionf Qm = SafeSlerp(Q2, Q1, ALPHA_DISTANCE);

          new_oculus_origin.linear() = Qm.matrix();

          rotation_state = STATE_MEDIUM_DIFF;
          position_state = STATE_MEDIUM_DIFF;
        }
        else
        {
          new_oculus_origin = m_motive_world_to_oculus_origin;
          position_state = STATE_SMALL_DIFF;
          rotation_state = STATE_SMALL_DIFF;
        }
      }

      oculus_origin_mat = new_oculus_origin;
    }

    oculus_mat = oculus_origin_mat * m_oculus_origin_to_oculus;

    prev_world_to_motive = m_world_to_motive_mat;
    prev_oculus_origin_to_oculus = m_oculus_origin_to_oculus;
  }

  void OculusOriginTracking::Reset()
  {
    prev_world_to_motive.linear() = Eigen::Matrix3f::Zero();
    prev_oculus_origin_to_oculus.linear() = Eigen::Matrix3f::Zero();
    position_state = STATE_INIT;
    rotation_state = STATE_INIT;
  }

  void OculusOriginTracking::Reset(const Eigen::Affine3f & initial_world_to_oculus,
                                   const Eigen::Affine3f & initial_oculus_origin_to_oculus)
  {
    prev_world_to_motive = initial_world_to_oculus;
    prev_oculus_origin_to_oculus = initial_oculus_origin_to_oculus;

    oculus_origin_mat = initial_world_to_oculus * initial_oculus_origin_to_oculus.inverse();
    oculus_mat = initial_oculus_origin_to_oculus;

    position_state = STATE_INIT;
    rotation_state = STATE_INIT;
  }

}
