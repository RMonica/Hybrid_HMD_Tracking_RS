/*
 * Copyright 2022-2024 Riccardo Monica
 * 
 * This software is distributed under the 3-clause BSD license.
 * You should have received a copy of the 3-clause BSD license
 * along with this software. If not, see
 * <https://opensource.org/license/bsd-3-clause>
 */

#ifndef CONSTRAINED_COMPLEMENTARY_TRACKING_H
#define CONSTRAINED_COMPLEMENTARY_TRACKING_H

#include "tracking_library.h"
#include "tracking_commons.h"
#include "tracking_interface.h"

#include <Eigen/Dense>
#include <stdint.h>
#include <cmath>
#include <iostream>
#include <memory>

namespace Tracking
{

// CONSTRAINED COMPLEMENTARY FILTER
class ConstrainedComplementaryTracking : public ITracking
{
  public:

  bool SetParameter(const int parameter_id, const float value) override;

  explicit ConstrainedComplementaryTracking(const bool correct_translation, const bool correct_rotation);

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

  bool HasConverged() const override
  {
    return has_converged;
  }

  void SetNotConverged() override
  {
    has_converged = false;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  private:

  Eigen::Affine3f oculus_origin_mat = Eigen::Affine3f::Identity();
  Eigen::Affine3f oculus_mat = Eigen::Affine3f::Identity();

  Eigen::Affine3f prev_world_to_motive = Eigen::Affine3f::Identity();
  Eigen::Affine3f prev_oculus_origin_to_oculus = Eigen::Affine3f::Identity();

  bool is_motive_lost = false;

  bool has_converged = false;

  bool correct_rotation;
  bool correct_translation;

  int position_state = STATE_INIT;
  int rotation_state = STATE_INIT;
};

}

#endif // CONSTRAINED_COMPLEMENTARY_TRACKING_H
