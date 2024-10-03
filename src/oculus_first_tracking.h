/*
 * Copyright 2022-2024 Riccardo Monica
 * 
 * This software is distributed under the 3-clause BSD license.
 * You should have received a copy of the 3-clause BSD license
 * along with this software. If not, see
 * <https://opensource.org/license/bsd-3-clause>
 */

#ifndef OCULUS_FIRST_TRACKING_H
#define OCULUS_FIRST_TRACKING_H

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

class OculusFirstTracking : public ITracking
{
  public:

  bool SetParameter(const int parameter_id, const float value) override
  {
    return false;
  }

  OculusFirstTracking()
  {
    Reset();
  }

  void Update(const Eigen::Affine3f & world_to_motive_mat,
              const Eigen::Affine3f & oculus_origin_to_oculus) override
  {
    if (prev_world_to_motive.linear() == Eigen::Matrix3f::Zero())
    {
      Reset(world_to_motive_mat, oculus_origin_to_oculus);
      return;
    }

    is_motive_lost = (world_to_motive_mat.linear() == prev_world_to_motive.linear()) &&
                     (world_to_motive_mat.translation() == prev_world_to_motive.translation());

    if (!is_motive_lost)
    {
      float angle = DistanceInDegrees(world_to_motive_mat, oculus_mat);
      float distance = DistanceInMeters(world_to_motive_mat, oculus_mat);

      const float large_angle_threshold = 30.0f;
      const float large_distance_threshold = 0.5f;
      if (distance > large_distance_threshold || angle > large_angle_threshold)
      {
        Reset(world_to_motive_mat, oculus_origin_to_oculus);

        rotation_state = STATE_REPOSITION;
        position_state = STATE_REPOSITION;
      }
    }
    else
    {
      position_state = STATE_SMALL_DIFF;
      rotation_state = STATE_SMALL_DIFF;
    }

    oculus_mat = oculus_origin_mat * oculus_origin_to_oculus;

    prev_world_to_motive = world_to_motive_mat;
    prev_oculus_origin_to_oculus = oculus_origin_to_oculus;
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

#endif // OCULUS_FIRST_TRACKING_H
