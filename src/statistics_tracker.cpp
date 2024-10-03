/*
 * Copyright 2022-2024 Riccardo Monica
 * 
 * This software is distributed under the 3-clause BSD license.
 * You should have received a copy of the 3-clause BSD license
 * along with this software. If not, see
 * <https://opensource.org/license/bsd-3-clause>
 */

#include "statistics_tracker.h"

namespace Tracking
{
  EXPORT_API void StatisticsTracker::Update(const Eigen::Affine3f & world_to_motive,
                                            const Eigen::Affine3f & world_to_oculus)
  {
    m_last_world_to_oculus.push_back(world_to_oculus);

    const Eigen::Affine3f diff = m_prev_world_to_oculus.inverse() * world_to_oculus;

    if (m_last_world_to_oculus.size() >= READ_PREV_FRAMES)
    {
      Eigen::Affine3f diff_30 = m_last_world_to_oculus[0].inverse() * world_to_oculus;
      diff_30 = PowAffine3f(diff_30, 1.0f / float(READ_PREV_FRAMES));

      Eigen::Affine3f diff_pow_30 = diff;
      //diff_pow_30.matrix() = diff.matrix().pow(float(READ_PREV_FRAMES));

      const Eigen::Affine3f diff_diff = diff_30 * diff_pow_30.inverse();

      const float dist_diff_rad = std::abs(Eigen::AngleAxisf(diff_diff.linear()).angle());
      const float dist_diff_met = diff_diff.translation().norm();

      m_last_vibration_rad = dist_diff_rad;
      m_last_vibration_met = dist_diff_met;

      m_last_world_to_oculus.erase(m_last_world_to_oculus.begin());
    }

    m_prev_world_to_motive = world_to_motive;
    m_prev_world_to_oculus = world_to_oculus;
  }
}
