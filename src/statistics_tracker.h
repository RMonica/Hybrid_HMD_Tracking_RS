/*
 * Copyright 2022-2024 Riccardo Monica
 * 
 * This software is distributed under the 3-clause BSD license.
 * You should have received a copy of the 3-clause BSD license
 * along with this software. If not, see
 * <https://opensource.org/license/bsd-3-clause>
 */

#ifndef STATISTICS_TRACKER_H
#define STATISTICS_TRACKER_H

#if _MSC_VER // this is defined when compiling with Visual Studio
#define EXPORT_API __declspec(dllexport) // Visual Studio needs annotating exported functions with this
#else
#define EXPORT_API
#endif

#include "tracking_commons.h"

#include <vector>
#include <stdint.h>
#include <iostream>

#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace Tracking
{
  class StatisticsTracker
  {
    public:
    typedef std::vector<float> FloatVector;
    typedef std::vector<Eigen::Affine3f> Affine3fVector;

    const int READ_PREV_FRAMES = 30;

    StatisticsTracker()
    {
      m_last_vibration_met = 0.0f;
      m_last_vibration_rad = 0.0f;
    }

    EXPORT_API void Update(const Eigen::Affine3f & world_to_motive,
                           const Eigen::Affine3f & world_to_oculus);

    float GetVibrationRad() const {return m_last_vibration_rad; }
    float GetVibrationMet() const {return m_last_vibration_met; }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    private:
    Affine3fVector m_last_world_to_oculus;

    float m_last_vibration_rad;
    float m_last_vibration_met;

    Eigen::Affine3f m_prev_world_to_motive;
    Eigen::Affine3f m_prev_world_to_oculus;
  };
}

#endif // STATISTICS_TRACKER_H
