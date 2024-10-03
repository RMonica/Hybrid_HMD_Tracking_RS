/*
 * Copyright 2022-2024 Riccardo Monica
 * 
 * This software is distributed under the 3-clause BSD license.
 * You should have received a copy of the 3-clause BSD license
 * along with this software. If not, see
 * <https://opensource.org/license/bsd-3-clause>
 */

#ifndef MOTIVE_FIRST_TRACKING_H
#define MOTIVE_FIRST_TRACKING_H

#include "tracking_library.h"
#include "tracking_commons.h"
#include "tracking_interface.h"

#include <stdint.h>
#include <cmath>
#include <iostream>
#include <memory>

#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace Tracking
{

class MotiveFirstTracking : public ITracking
{
  public:

  bool SetParameter(const int parameter_id, const float value) override
  {
    return false;
  }

  MotiveFirstTracking()
  {
    Reset();
  }

  void Update(const Eigen::Affine3f & m_world_to_motive_mat,
              const Eigen::Affine3f & m_oculus_origin_to_oculus) override
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
      Eigen::Affine3f new_oculus_origin;

      new_oculus_origin = m_motive_world_to_oculus_origin;
      position_state = STATE_SMALL_DIFF;
      rotation_state = STATE_SMALL_DIFF;

      oculus_origin_mat = new_oculus_origin;
    }

    oculus_mat = oculus_origin_mat * m_oculus_origin_to_oculus;

    prev_world_to_motive = m_world_to_motive_mat;
    prev_oculus_origin_to_oculus = m_oculus_origin_to_oculus;
  }

  void Reset() override
  {
    prev_world_to_motive.linear() = Eigen::Matrix3f::Zero();
    prev_oculus_origin_to_oculus.linear() = Eigen::Matrix3f::Zero();
    position_state = STATE_INIT;
    rotation_state = STATE_INIT;
  }

  void Reset(const Eigen::Affine3f & initial_world_to_oculus,
             const Eigen::Affine3f & initial_oculus_origin_to_oculus) override
  {
    prev_world_to_motive = initial_world_to_oculus;
    prev_oculus_origin_to_oculus = initial_oculus_origin_to_oculus;

    oculus_origin_mat = initial_world_to_oculus * initial_oculus_origin_to_oculus.inverse();
    oculus_mat = initial_world_to_oculus;

    position_state = STATE_INIT;
    rotation_state = STATE_INIT;
  }

  bool IsMotiveLost() const override
  {
    return is_motive_lost;
  }

  Eigen::Affine3f GetEstimatedOculus() const override
  {
    return oculus_mat;
  }

  int GetPositionStatus() const override
  {
    return position_state;
  }

  int GetRotationStatus() const override
  {
    return rotation_state;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  private:

  Eigen::Affine3f oculus_origin_mat = Eigen::Affine3f::Identity();
  Eigen::Affine3f oculus_mat = Eigen::Affine3f::Identity();

  Eigen::Affine3f prev_world_to_motive = Eigen::Affine3f::Identity();
  Eigen::Affine3f prev_oculus_origin_to_oculus = Eigen::Affine3f::Identity();

  bool is_motive_lost = false;

  int position_state = STATE_INIT;
  int rotation_state = STATE_INIT;
};

}

#endif // MOTIVE_FIRST_TRACKING_H
