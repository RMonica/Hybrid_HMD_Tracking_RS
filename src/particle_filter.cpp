/*
 * Copyright 2022-2024 Riccardo Monica
 * 
 * This software is distributed under the 3-clause BSD license.
 * You should have received a copy of the 3-clause BSD license
 * along with this software. If not, see
 * <https://opensource.org/license/bsd-3-clause>
 */

#include "particle_filter.h"

#include <random>

namespace Tracking
{

Eigen::Affine3f ParticleFilter::Particle::ToAffine3f() const
{
  Eigen::Affine3f result;
  result.linear() = orientation.matrix();
  result.translation() = position;
  return result;
}

ParticleFilter::Particle ParticleFilter::Particle::FromAffine3f(const Eigen::Affine3f & a)
{
  Particle result;
  result.orientation = Eigen::Quaternionf(a.linear());
  result.position = a.translation();
  return result;
}

ParticleFilter::Particle ParticleFilter::AverageParticles(const ParticleVector & particles,
                                                          const FloatVector & weights)
{
  Eigen::Vector3f avg_transl = Eigen::Vector3f::Zero();
  Eigen::Vector4f avg_rot = Eigen::Vector4f::Zero();

  float sum = 0.0f;
  for (uint64 i = 0; i < particles.size(); i++)
  {
    const Particle & p = particles[i];
    const float weight = weights.empty() ? 1.0f : weights[i];
    avg_transl += p.position * weight;
    avg_rot += Eigen::Vector4f(p.orientation.x(), p.orientation.y(), p.orientation.z(), p.orientation.w()) * weight;
    sum += weight;
  }
  if (sum > 0.0f)
  {
    avg_transl /= sum;
    avg_rot.normalize();
  }

  Particle result(avg_transl, Eigen::Quaternionf(avg_rot));
  return result;
}

ParticleFilter::ParticleFilter(): m_random_generator(std::random_device()())
{
  SetParameter(PARAM_NUM_PARTICLES, 200);
  SetParameter(PARAM_SIGMA_TRANSL, 0.1f);
  SetParameter(PARAM_SIGMA_ROT, 2.0f / 180.0f * float(M_PI));
  SetParameter(PARAM_PSIGMA_TRANSL, 0.005f);
  SetParameter(PARAM_PSIGMA_ROT, 2.0f / 180.0f * float(M_PI));
  Reset();
}

void ParticleFilter::Reset()
{
  prev_world_to_motive.linear() = Eigen::Matrix3f::Zero();
  oculus_mat = Eigen::Affine3f::Identity();
  prev_oculus_origin_to_oculus = Eigen::Affine3f::Identity();
  position_state = STATE_INIT;
  rotation_state = STATE_INIT;

  m_random_generator.seed(134); // default seed

  m_particles.clear();
}

void ParticleFilter::Reset(const Eigen::Affine3f & initial_world_to_oculus,
                           const Eigen::Affine3f & initial_oculus_origin_to_oculus)
{
  prev_world_to_motive = initial_world_to_oculus;

  prev_oculus_origin_to_oculus = initial_oculus_origin_to_oculus;
  oculus_mat = initial_oculus_origin_to_oculus;

  position_state = STATE_INIT;
  rotation_state = STATE_INIT;

  const uint64 num_particles = uint64(m_params[PARAM_NUM_PARTICLES]);
  m_particles.clear();
  for (uint64 i = 0; i < num_particles; i++)
    m_particles.push_back(Particle::FromAffine3f(oculus_mat));
}

Eigen::Affine3f ParticleFilter::GetNoise(const float sigma_transl, const float sigma_rot)
{
  Eigen::Affine3f result;

  std::uniform_real_distribution<> gen_axis(-1.0f, 1.0f);

  Eigen::Vector3f axis;
  axis.x() = float(gen_axis(m_random_generator));
  axis.y() = float(gen_axis(m_random_generator));
  axis.z() = float(gen_axis(m_random_generator));
  if (axis.squaredNorm() <= 0.0001f)
    axis = Eigen::Vector3f::UnitX();
  axis.normalize();

  std::normal_distribution<> gen_angle(0.0f, sigma_rot);
  const float angle = float(gen_angle(m_random_generator));

  std::normal_distribution<> gen_transl(0.0f, sigma_transl);
  Eigen::Vector3f transl(float(gen_transl(m_random_generator)),
                         float(gen_transl(m_random_generator)),
                         float(gen_transl(m_random_generator)));

  result.linear() = Eigen::AngleAxisf(angle, axis).matrix();
  result.translation() = transl;

  return result;
}

void ParticleFilter::Update(const Eigen::Affine3f & m_world_to_motive_mat,
                            const Eigen::Affine3f & m_oculus_origin_to_oculus)
{
  if (prev_world_to_motive.linear() == Eigen::Matrix3f::Zero())
  {
    prev_world_to_motive = Eigen::Affine3f::Identity();
    prev_oculus_origin_to_oculus = Eigen::Affine3f::Identity();
    oculus_mat = Eigen::Affine3f::Identity();
    position_state = STATE_INIT;
    rotation_state = STATE_INIT;
    return;
  }

  is_motive_lost = (m_world_to_motive_mat.linear() == prev_world_to_motive.linear()) &&
                   (m_world_to_motive_mat.translation() == prev_world_to_motive.translation());

  const uint64 num_particles = uint64(m_params[PARAM_NUM_PARTICLES]);
  const float sigma_transl = m_params[PARAM_SIGMA_TRANSL];
  const float sigma_rot = m_params[PARAM_SIGMA_ROT];
  const float psigma_transl = m_params[PARAM_PSIGMA_TRANSL];
  const float psigma_rot = m_params[PARAM_PSIGMA_ROT];

  oculus_mat = oculus_mat * prev_oculus_origin_to_oculus.inverse() * m_oculus_origin_to_oculus;

  // prediction step
  {
    for (uint64 i = 0; i < m_particles.size(); i++)
    {
      Particle & particle = m_particles[i];
      Eigen::Affine3f mat = particle.ToAffine3f();

      mat = mat * prev_oculus_origin_to_oculus.inverse() * m_oculus_origin_to_oculus;

      mat = mat * GetNoise(psigma_transl, psigma_rot);

      particle = Particle::FromAffine3f(mat);
    }
  }

  if (!is_motive_lost)
  {
    Eigen::Affine3f m_world_to_oculus = oculus_mat;

    float distance = DistanceInMeters(m_world_to_motive_mat, m_world_to_oculus);
    float angle = DistanceInDegrees(m_world_to_motive_mat, m_world_to_oculus);

    const float large_angle_threshold = 30.0f;
    const float large_distance_threshold = 0.5f;
    // reposition
    if (distance > large_distance_threshold || angle > large_angle_threshold ||
        m_particles.empty())
    {
      oculus_mat = m_world_to_motive_mat;
      prev_oculus_origin_to_oculus = m_world_to_motive_mat;

      m_particles.clear();
      for (uint64 i = 0; i < num_particles; i++)
        m_particles.push_back(Particle::FromAffine3f(oculus_mat));

      if (angle > large_angle_threshold)
        rotation_state = STATE_REPOSITION;
      if (distance > large_distance_threshold)
        position_state = STATE_REPOSITION;
    }
    else
    {
      // correction step
      FloatVector observation_probs(m_particles.size());
      uint64 max_particle = 0;
      float max_particle_prob = 0.0f;
      for (uint64 i = 0; i < m_particles.size(); i++)
      {
        const Particle & particle = m_particles[i];
        const Eigen::Affine3f p = particle.ToAffine3f();

        float pos_dist = DistanceInMeters(m_world_to_motive_mat, p);
        float angle_dist = DistanceInRadians(m_world_to_motive_mat, p);
        float observation_prob = float(std::exp(-0.5 * SQR(pos_dist) / SQR(sigma_transl)) *
                                       std::exp(-0.5 * SQR(angle_dist) / SQR(sigma_rot)));
        observation_probs[i] = observation_prob;

        if (observation_prob > max_particle_prob)
        {
          max_particle_prob = observation_prob;
          max_particle = i;
        }
      }
      float sum_probs = 0.0f;
      for (float p : observation_probs)
        sum_probs += p;

      if (sum_probs <= 0.001) // prevent division-by-zero and reinit
      {
        oculus_mat = m_world_to_motive_mat;

        m_particles.clear();
        for (uint64 i = 0; i < num_particles; i++)
          m_particles.push_back(Particle::FromAffine3f(oculus_mat));

        position_state = STATE_REPOSITION;
        rotation_state = STATE_REPOSITION;
      }
      else // update
      {
        ParticleVector new_particles;
        std::uniform_real_distribution<> gen_extracted_num(0.0f, sum_probs);
        new_particles.push_back(m_particles[max_particle]);
        for (uint64 i = 1; i < num_particles; i++)
        {
          float prob_counter = 0.0f;
          const float extracted_num = float(gen_extracted_num(m_random_generator));
          for (uint64 h = 0; h < m_particles.size(); h++)
          {
            prob_counter += observation_probs[h];
            if (prob_counter >= extracted_num)
            {
              new_particles.push_back(m_particles[h]);
              break;
            }
          }
        }

        const Particle avg_particle = AverageParticles(m_particles, observation_probs);
        oculus_mat = avg_particle.ToAffine3f();

        m_particles.swap(new_particles);

        position_state = STATE_SMALL_DIFF;
        rotation_state = STATE_SMALL_DIFF;
      }

      prev_oculus_origin_to_oculus = m_oculus_origin_to_oculus;
    }
  }

  oculus_mat.linear() = ApproxNormalizeRotation(oculus_mat.linear());

  prev_oculus_origin_to_oculus = m_oculus_origin_to_oculus;

  prev_world_to_motive = m_world_to_motive_mat;
}

}
