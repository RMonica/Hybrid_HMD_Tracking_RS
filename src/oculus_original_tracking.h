/*
 * Copyright 2022-2024 Riccardo Monica
 * 
 * This software is distributed under the 3-clause BSD license.
 * You should have received a copy of the 3-clause BSD license
 * along with this software. If not, see
 * <https://opensource.org/license/bsd-3-clause>
 */

#ifndef ORIGINAL_TRACKING_H
#define ORIGINAL_TRACKING_H

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

class OculusOriginTracking : public ITracking
{
  public:

  bool SetParameter(const int parameter_id, const float value) override;

  OculusOriginTracking();

  void Update(const Eigen::Affine3f & m_world_to_motive_mat,
              const Eigen::Affine3f & m_oculus_origin_to_oculus) override;

  void Reset() override;

  void Reset(const Eigen::Affine3f & initial_world_to_oculus,
             const Eigen::Affine3f & initial_oculus_origin_to_oculus) override;

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

#endif // ORIGINAL_TRACKING_H
