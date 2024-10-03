/*
 * Copyright 2022-2024 Riccardo Monica
 * 
 * This software is distributed under the 3-clause BSD license.
 * You should have received a copy of the 3-clause BSD license
 * along with this software. If not, see
 * <https://opensource.org/license/bsd-3-clause>
 */

#include "tracking_library.h"
#include "tracking_commons.h"
#include "oculus_original_tracking.h"
#include "constrained_complementary_tracking.h"
#include "complementary_tracking.h"
#include "tracking_interface.h"
#include "motive_first_tracking.h"
#include "oculus_first_tracking.h"
#include "double_exponential_smoothing.h"
#include "exponential_smoothing.h"
#include "particle_filter.h"
#include "kalman_tracking.h"
#include "kalman_tracking_constrained.h"
#include "directed_complementary_tracking.h"
#include "kalman_tracking_wrapper.h"

#include <Eigen/Dense>
#include <stdint.h>
#include <cmath>
#include <iostream>
#include <memory>

namespace Tracking
{
  ITracking::Ptr current_tracker(new ConstrainedComplementaryTracking(true, true));
}

extern "C"
{
  EXPORT_API void TrackingUpdate(float * motive_mat,
                                 float * oculus_mat)
  {
    Tracking::current_tracker->Update(Tracking::FloatvToAffine3f(motive_mat), Tracking::FloatvToAffine3f(oculus_mat));
  }

  EXPORT_API bool TrackingIsMotiveLost()
  {
    return Tracking::current_tracker->IsMotiveLost();
  }

  EXPORT_API void TrackingReset()
  {
    Tracking::current_tracker->Reset();
  }

  EXPORT_API void TrackingResetWithInit(float * world_to_oculus_mat,
                                        float * world_to_oculus_origin_mat)
  {
    Tracking::current_tracker->Reset(Tracking::FloatvToAffine3f(world_to_oculus_mat),
                                     Tracking::FloatvToAffine3f(world_to_oculus_origin_mat));
  }

  EXPORT_API void TrackingGetEstimatedOculus(float * oculus_mat_out)
  {
    Eigen::Affine3f oculus_mat = Tracking::current_tracker->GetEstimatedOculus();
    Tracking::Affine3fToFloatv(oculus_mat, oculus_mat_out);
  }

  EXPORT_API int32_t TrackingGetPositionStatus()
  {
    return Tracking::current_tracker->GetPositionStatus();
  }

  EXPORT_API int32_t TrackingGetRotationStatus()
  {
    return Tracking::current_tracker->GetRotationStatus();
  }

  EXPORT_API void TrackingSetKalmanWrappedMode(const int mode)
  {
    TrackingSetMode(mode);
    Tracking::current_tracker.reset(new Tracking::KalmanTrackingWrapper(Tracking::current_tracker));
  }

  EXPORT_API void TrackingSetMode(const int mode)
  {
    if (mode == Tracking::Mode::MODE_OCULUS_ORIGIN)
      Tracking::current_tracker.reset(new Tracking::OculusOriginTracking);
    else if (mode == Tracking::Mode::MODE_CONSTR_COMPLEMENTARY)
      Tracking::current_tracker.reset(new Tracking::ConstrainedComplementaryTracking(true, true));
    else if (mode == Tracking::Mode::MODE_CONSTR_COMPLEMENTARY_RO)
      Tracking::current_tracker.reset(new Tracking::ConstrainedComplementaryTracking(false, true));
    else if (mode == Tracking::Mode::MODE_CONSTR_COMPLEMENTARY_TO)
      Tracking::current_tracker.reset(new Tracking::ConstrainedComplementaryTracking(true, false));
    else if (mode == Tracking::Mode::MODE_COMPLEMENTARY)
      Tracking::current_tracker.reset(new Tracking::ComplementaryTracking);
    else if (mode == Tracking::Mode::MODE_MOTIVE_FIRST)
      Tracking::current_tracker.reset(new Tracking::MotiveFirstTracking);
    else if (mode == Tracking::Mode::MODE_DOUBLE_EXP)
      Tracking::current_tracker.reset(new Tracking::DoubleExponentialSmoothing);
    else if (mode == Tracking::Mode::MODE_EXP)
      Tracking::current_tracker.reset(new Tracking::ExponentialSmoothing);
    else if (mode == Tracking::Mode::MODE_PARTICLE_FILTER)
      Tracking::current_tracker.reset(new Tracking::ParticleFilter);
    else if (mode == Tracking::Mode::MODE_KALMAN_FILTER)
      Tracking::current_tracker.reset(new Tracking::KalmanTracking);
    else if (mode == Tracking::Mode::MODE_CONSTR_KALMAN_FILTER)
      Tracking::current_tracker.reset(new Tracking::ConstrainedKalmanTracking);
    else if (mode == Tracking::Mode::MODE_DIR_COMPLEMENTARY)
      Tracking::current_tracker.reset(new Tracking::DirectedComplementaryTracking);
    else if (mode == Tracking::Mode::MODE_OCULUS_FIRST)
      Tracking::current_tracker.reset(new Tracking::OculusFirstTracking);
    else
      std::cout << "TrackingSetMode: Error: unknown mode " << mode << std::endl;
  }

  EXPORT_API bool TrackingSetParameter(const int parameter_id, const float value)
  {
    return Tracking::current_tracker->SetParameter(parameter_id, value);
  }

  EXPORT_API float TrackingGetParameter(const int parameter_id)
  {
    return Tracking::current_tracker->GetParameter(parameter_id);
  }

  EXPORT_API bool TrackingHasConverged()
  {
    return Tracking::current_tracker->HasConverged();
  }
}
