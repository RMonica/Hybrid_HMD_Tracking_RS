/*
 * Copyright 2022-2024 Riccardo Monica
 * 
 * This software is distributed under the 3-clause BSD license.
 * You should have received a copy of the 3-clause BSD license
 * along with this software. If not, see
 * <https://opensource.org/license/bsd-3-clause>
 */

#include "constrained_complementary_tracking.h"

namespace Tracking
{

ConstrainedComplementaryTracking::ConstrainedComplementaryTracking(const bool correct_translation, const bool correct_rotation)
{
  SetParameter(PARAM_ROTATION_LIMIT, 0.019630f);
  SetParameter(PARAM_TRANSLATION_LIMIT, 0.000130f);

  SetParameter(PARAM_ROTATION_DYN_LIMIT, 0.036499f);
  SetParameter(PARAM_ROTATION_CURV_LIMIT, 1.579485f); // degrees / meter
  SetParameter(PARAM_TRANSLATION_DYN_LIMIT, 0.092148f);

  SetParameter(PARAM_MULT, 1.0f);

  this->correct_rotation = correct_rotation;
  this->correct_translation = correct_translation;

  Reset();
}

bool ConstrainedComplementaryTracking::SetParameter(const int parameter_id, const float value)
{
  switch (parameter_id)
  {
    case PARAM_ROTATION_LIMIT:
    case PARAM_ROTATION_DYN_LIMIT:
    case PARAM_ROTATION_CURV_LIMIT:
    case PARAM_TRANSLATION_DYN_LIMIT:
    case PARAM_TRANSLATION_LIMIT:
    case PARAM_MULT:
      return ITracking::SetParameter(parameter_id, value);
    default:
      return false;
  }
}

void ConstrainedComplementaryTracking::Reset(const Eigen::Affine3f & initial_world_to_oculus,
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

void ConstrainedComplementaryTracking::Reset()
{
  prev_world_to_motive.linear() = Eigen::Matrix3f::Zero();
  prev_oculus_origin_to_oculus.linear() = Eigen::Matrix3f::Zero();
  position_state = STATE_INIT;
  rotation_state = STATE_INIT;

  has_converged = false;
}

void ConstrainedComplementaryTracking::Update(const Eigen::Affine3f & m_world_to_motive_mat,
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

    const float mult = m_params[PARAM_MULT];

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

      if (correct_rotation && correct_translation)
      {
        const float rotation_limit = std::max(m_params[PARAM_ROTATION_LIMIT],
                                     std::max(m_params[PARAM_ROTATION_DYN_LIMIT] * mult * oculus_angle_diff,
                                              m_params[PARAM_ROTATION_CURV_LIMIT] * mult * oculus_dist_diff));
        const float translation_limit = std::max(m_params[PARAM_TRANSLATION_LIMIT],
                                                 m_params[PARAM_TRANSLATION_DYN_LIMIT] * mult * oculus_dist_diff);

        new_oculus = m_world_to_oculus;

        float frameNumbers = (distance / translation_limit + angle / rotation_limit);
        if (false)
        {
          const float P = 0.07;
          const float rotation_tot_limit_P = rotation_limit / P;
          const float translation_tot_limit_P = translation_limit / P;

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
      }
      else
      {
        if (correct_rotation)
        {
          const float rotation_limit = m_params[PARAM_ROTATION_LIMIT] +
                                       m_params[PARAM_ROTATION_DYN_LIMIT] * mult * oculus_angle_diff +
                                       m_params[PARAM_ROTATION_CURV_LIMIT] * mult * oculus_dist_diff;

          if (angle > rotation_limit && !has_converged)
          {
            float frameNumbers = angle / rotation_limit;
            float ALPHA = 1 / frameNumbers;

            new_oculus.translation() = m_world_to_oculus.translation();
            new_oculus.linear() = InterpolateRotation(m_world_to_oculus.linear(), m_world_to_motive_mat.linear(), ALPHA);

            position_state = STATE_MEDIUM_DIFF;
            rotation_state = STATE_MEDIUM_DIFF;
          }
          else
          {
            new_oculus.translation() = m_world_to_oculus.translation();
            new_oculus.linear() = m_world_to_motive_mat.linear();
            position_state = STATE_SMALL_DIFF;
            rotation_state = STATE_SMALL_DIFF;

            has_converged = true;
          }
        }

        if (correct_translation)
        {
          const float translation_limit = m_params[PARAM_TRANSLATION_LIMIT] +
                                          m_params[PARAM_TRANSLATION_DYN_LIMIT] * mult * oculus_dist_diff;

          if (distance > translation_limit && !has_converged)
          {
            float frameNumbers = distance / translation_limit;
            float ALPHA = 1 / frameNumbers;

            new_oculus.linear() = m_world_to_oculus.linear();
            new_oculus.translation() = InterpolateTranslation(m_world_to_oculus.translation(),
                                                              m_world_to_motive_mat.translation(), ALPHA);

            position_state = STATE_MEDIUM_DIFF;
            rotation_state = STATE_MEDIUM_DIFF;
          }
          else
          {
            new_oculus.linear() = m_world_to_oculus.linear();
            new_oculus.translation() = m_world_to_motive_mat.translation();
            position_state = STATE_SMALL_DIFF;
            rotation_state = STATE_SMALL_DIFF;

            has_converged = true;
          }
        }
      }
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
