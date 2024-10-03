/*
 * Copyright 2022-2024 Riccardo Monica
 * 
 * This software is distributed under the 3-clause BSD license.
 * You should have received a copy of the 3-clause BSD license
 * along with this software. If not, see
 * <https://opensource.org/license/bsd-3-clause>
 */

#ifndef TRACKING_LIBRARY_H
#define TRACKING_LIBRARY_H

#if _MSC_VER // this is defined when compiling with Visual Studio
  #define EXPORT_API __declspec(dllexport) // Visual Studio needs annotating exported functions with this
#else
  #define EXPORT_API
#endif

#include <Eigen/Dense>
namespace Tracking
{
  EXPORT_API Eigen::Affine3f FloatvToAffine3f(const float * mat);
  EXPORT_API void Affine3fToFloatv(const Eigen::Affine3f & s, float * d);

  enum Mode
  {
    MODE_OCULUS_ORIGIN        = 0,
    MODE_CONSTR_COMPLEMENTARY = 1,
    MODE_COMPLEMENTARY        = 2,
    MODE_MOTIVE_FIRST         = 3,
    MODE_DOUBLE_EXP           = 4,
    MODE_EXP                  = 5, // running average
    MODE_PARTICLE_FILTER      = 6,
    MODE_KALMAN_FILTER        = 7,
    MODE_CONSTR_KALMAN_FILTER = 8,
    MODE_DIR_COMPLEMENTARY    = 9,
    MODE_OCULUS_FIRST         = 10,
    MODE_CONSTR_COMPLEMENTARY_RO   = 11, // rotation only
    MODE_CONSTR_COMPLEMENTARY_TO   = 12, // translation only
  };
}

extern "C"
{
  EXPORT_API void TrackingUpdate(float * motive_mat,
                                 float * oculus_mat);

  EXPORT_API void TrackingGetEstimatedOculus(float * oculus_mat_out);

  EXPORT_API void TrackingReset();
  EXPORT_API void TrackingResetWithInit(float * world_to_oculus_mat,
                                        float * oculus_origin_to_oculus_mat);

  EXPORT_API bool TrackingIsMotiveLost();

  EXPORT_API bool TrackingHasConverged();

  EXPORT_API int32_t TrackingGetPositionStatus();
  EXPORT_API int32_t TrackingGetRotationStatus();

  EXPORT_API void TrackingSetMode(const int mode);
  EXPORT_API void TrackingSetKalmanWrappedMode(const int mode);

  EXPORT_API bool TrackingSetParameter(const int parameter_id, const float value);
  EXPORT_API float TrackingGetParameter(const int parameter_id);
}

#endif // TRACKING_LIBRARY_H
