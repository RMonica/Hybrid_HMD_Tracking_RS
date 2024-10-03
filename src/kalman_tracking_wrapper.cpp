/*
 * Copyright 2022-2024 Riccardo Monica
 * 
 * This software is distributed under the 3-clause BSD license.
 * You should have received a copy of the 3-clause BSD license
 * along with this software. If not, see
 * <https://opensource.org/license/bsd-3-clause>
 */

#include "kalman_tracking_wrapper.h"

#include "ekf_pose3_tracker.h"

#define PI 3.14159265359

template <typename T>
T SQR(const T & t) {return t * t; }

namespace Tracking
{

static ekfpt::EkfPose3Tracker::SE3 EigenToSE3(const Eigen::Affine3f & m)
{
  Eigen::Matrix4d matr = m.cast<double>().matrix();
  Eigen::Isometry3d transfEigen;
  transfEigen = matr;
  return transfEigen;
}

static Eigen::Affine3f SE3ToEigen(const ekfpt::EkfPose3Tracker::SE3 & se3)
{
  Eigen::Affine3f result;
  result.matrix() = se3.isometry().matrix().cast<float>();
  return result;
}

float KalmanTrackingWrapper::GetParameter(const int parameter_id)
{
  float p = ITracking::GetParameter(parameter_id);
  if (p != -1.0f)
    return p;
  return m_wrapped_tracker->GetParameter(parameter_id);
}

bool KalmanTrackingWrapper::SetParameter(const int parameter_id, const float value)
{
  switch (parameter_id)
  {
    case PARAM_W_SIGMA_ROT:
    case PARAM_W_SIGMA_TRANSL:
    case PARAM_W_PSIGMA_TRANSL:
    case PARAM_W_PSIGMA_ROT:
    case PARAM_W_ISIGMA_TRANSL:
    case PARAM_W_ISIGMA_ROT:
      return ITracking::SetParameter(parameter_id, value);
    default:
      return m_wrapped_tracker->SetParameter(parameter_id, value);
  }
}

void KalmanTrackingWrapper::Reset()
{
  m_wrapped_tracker->Reset();
  prev_world_to_motive.linear() = Eigen::Matrix3f::Zero();
  prev_oculus_origin_to_oculus_mat.linear() = Eigen::Matrix3f::Zero();

  is_motive_lost = false;
}

void KalmanTrackingWrapper::Reset(const Eigen::Affine3f & initial_world_to_oculus,
                                  const Eigen::Affine3f & initial_oculus_origin_to_oculus)
{
  m_wrapped_tracker->Reset(initial_world_to_oculus,
                           initial_oculus_origin_to_oculus);
  prev_world_to_motive = initial_world_to_oculus;
  prev_oculus_origin_to_oculus_mat = initial_oculus_origin_to_oculus;

  pred_oculus_mat = initial_world_to_oculus;

  ekfpt::EkfPose3Tracker & tracker = *m_ekf_pose3_tracker;

  ekfpt::EkfPose3Tracker::SE3 stateMean = EigenToSE3(initial_world_to_oculus);

  const double sigma_transl = SQR(m_params[PARAM_W_ISIGMA_TRANSL]);
  const double sigma_rot = SQR(m_params[PARAM_W_ISIGMA_ROT]);

  ekfpt::EkfPose3Tracker::Vector6 stdevInit;
  stdevInit << sigma_transl, sigma_transl, sigma_transl, sigma_rot, sigma_rot, sigma_rot;
  const ekfpt::EkfPose3Tracker::Matrix6 covarInit = stdevInit.asDiagonal();

  is_motive_lost = false;

  tracker.initState(stateMean, covarInit);
}

KalmanTrackingWrapper::KalmanTrackingWrapper(const ITracking::Ptr wrapped_tracker):
  m_wrapped_tracker(wrapped_tracker)
{
  if (!m_wrapped_tracker)
    std::exit(1); // should never happen

  m_ekf_pose3_tracker.reset(new ekfpt::EkfPose3Tracker);

//  SetParameter(PARAM_W_SIGMA_ROT, 1.0f / 180.0f * float(PI));
//  SetParameter(PARAM_W_SIGMA_TRANSL, 0.05f);

//  SetParameter(PARAM_W_PSIGMA_ROT, 0.1f / 180.0f * float(PI));
//  SetParameter(PARAM_W_PSIGMA_TRANSL, 0.01f);

//  SetParameter(PARAM_W_ISIGMA_ROT, 1.0f / 180.0f * float(PI));
//  SetParameter(PARAM_W_ISIGMA_TRANSL, 0.1f);

  SetParameter(PARAM_W_SIGMA_ROT, 5.0f / 180.0f * float(PI));
  SetParameter(PARAM_W_SIGMA_TRANSL, 0.01f);

  SetParameter(PARAM_W_PSIGMA_ROT, 0.1f / 180.0f * float(PI));
  SetParameter(PARAM_W_PSIGMA_TRANSL, 0.01f);

  SetParameter(PARAM_W_ISIGMA_ROT, 5.0f / 180.0f * float(PI));
  SetParameter(PARAM_W_ISIGMA_TRANSL, 0.01f);
  Reset();
}

void KalmanTrackingWrapper::Update(const Eigen::Affine3f & world_to_motive_mat,
                            const Eigen::Affine3f & oculus_origin_to_oculus_mat)
{
  ekfpt::EkfPose3Tracker & tracker = *m_ekf_pose3_tracker;

  if (prev_world_to_motive.linear() == Eigen::Matrix3f::Zero())
  {
    Reset(world_to_motive_mat, oculus_origin_to_oculus_mat);
    return;
  }

  bool prev_is_motive_lost = is_motive_lost;

  is_motive_lost = (world_to_motive_mat.linear() == prev_world_to_motive.linear()) &&
                   (world_to_motive_mat.translation() == prev_world_to_motive.translation());

  ekfpt::EkfPose3Tracker::Vector6 stdevOdom;
  const double psigma_transl = SQR(m_params[PARAM_W_PSIGMA_TRANSL]);
  const double psigma_rot = SQR(m_params[PARAM_W_PSIGMA_ROT]);
  stdevOdom << psigma_transl, psigma_transl, psigma_transl, psigma_rot, psigma_rot, psigma_rot;
  ekfpt::EkfPose3Tracker::Matrix6 covarOdom = stdevOdom.asDiagonal();

  const Eigen::Affine3f oculus_diff = prev_oculus_origin_to_oculus_mat.inverse() * oculus_origin_to_oculus_mat;
  const ekfpt::EkfPose3Tracker::SE3 diffOdom = EigenToSE3(oculus_diff);

  tracker.updatePrediction(diffOdom, covarOdom);

  if (!is_motive_lost)
  {
    pred_oculus_mat = SE3ToEigen(tracker.getMean());

    const float distance = DistanceInMeters(world_to_motive_mat, pred_oculus_mat);
    const float angle = DistanceInDegrees(world_to_motive_mat, pred_oculus_mat);

    if (distance > 0.5f || angle > 30.0f)
    {
      // error too large, reinit
      ekfpt::EkfPose3Tracker::SE3 stateMean = EigenToSE3(world_to_motive_mat);

      const double sigma_transl = SQR(m_params[PARAM_W_ISIGMA_TRANSL]);
      const double sigma_rot = SQR(m_params[PARAM_W_ISIGMA_ROT]);

      ekfpt::EkfPose3Tracker::Vector6 stdevInit;
      stdevInit << sigma_transl, sigma_transl, sigma_transl, sigma_rot, sigma_rot, sigma_rot;
      const ekfpt::EkfPose3Tracker::Matrix6 covarInit = stdevInit.asDiagonal();

      tracker.initState(stateMean, covarInit);
    }
    else
    {
      const double sigma_transl = SQR(m_params[PARAM_W_SIGMA_TRANSL]);
      const double sigma_rot = SQR(m_params[PARAM_W_SIGMA_ROT]);

      ekfpt::EkfPose3Tracker::Vector6 stdevMeas;
      stdevMeas << sigma_transl, sigma_transl, sigma_transl, sigma_rot, sigma_rot, sigma_rot;
      ekfpt::EkfPose3Tracker::Matrix6 covarMeas = stdevMeas.asDiagonal();

      ekfpt::EkfPose3Tracker::SE3 transfMeas = EigenToSE3(world_to_motive_mat);

      tracker.updateCorrection(transfMeas, covarMeas);
    }
  }

  pred_oculus_mat = SE3ToEigen(tracker.getMean());

  prev_world_to_motive = world_to_motive_mat;
  prev_oculus_origin_to_oculus_mat = oculus_origin_to_oculus_mat;

  if (!is_motive_lost && prev_is_motive_lost)
  {
    m_wrapped_tracker->SetNotConverged();
  }

  m_wrapped_tracker->Update(pred_oculus_mat, oculus_origin_to_oculus_mat);
}

}
