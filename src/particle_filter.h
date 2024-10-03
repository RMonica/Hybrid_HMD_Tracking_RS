/*
 * Copyright 2022-2024 Riccardo Monica
 * 
 * This software is distributed under the 3-clause BSD license.
 * You should have received a copy of the 3-clause BSD license
 * along with this software. If not, see
 * <https://opensource.org/license/bsd-3-clause>
 */

#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include "tracking_library.h"
#include "tracking_commons.h"
#include "tracking_interface.h"

#include <stdint.h>
#include <cmath>
#include <iostream>
#include <memory>
#include <random>

#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace Tracking
{

class ParticleFilter : public ITracking
{
  public:

  struct Particle
  {
    Eigen::Vector3f    position;
    Eigen::Quaternionf orientation;

    Particle(const Eigen::Vector3f & p, const Eigen::Quaternionf & o): position(p), orientation(o) {}
    Particle() {position = Eigen::Vector3f::Zero(); orientation = Eigen::Quaternionf::Identity(); }

    Eigen::Affine3f ToAffine3f() const;
    static Particle FromAffine3f(const Eigen::Affine3f & a);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };
  typedef std::vector<Particle, Eigen::aligned_allocator<Particle> > ParticleVector;
  typedef std::vector<float> FloatVector;

  bool SetParameter(const int parameter_id, const float value) override
  {
    switch (parameter_id)
    {
      case PARAM_NUM_PARTICLES:
      case PARAM_SIGMA_ROT:
      case PARAM_SIGMA_TRANSL:
      case PARAM_PSIGMA_TRANSL:
      case PARAM_PSIGMA_ROT:
        return ITracking::SetParameter(parameter_id, value);
      default:
        return false;
    }
  }

  ParticleFilter();

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

  Eigen::Affine3f GetNoise(const float sigma_transl, const float sigma_rot);

  static Particle AverageParticles(const ParticleVector & particles, const FloatVector & weights = FloatVector());

  ParticleVector m_particles;

  Eigen::Affine3f oculus_mat = Eigen::Affine3f::Identity();

  Eigen::Affine3f prev_world_to_motive = Eigen::Affine3f::Identity();

  Eigen::Affine3f prev_oculus_origin_to_oculus = Eigen::Affine3f::Identity();

  bool is_motive_lost = false;

  int position_state = STATE_INIT;
  int rotation_state = STATE_INIT;

  std::minstd_rand m_random_generator;
};

}

#endif // PARTICLE_FILTER_H
