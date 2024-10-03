/*
 * Copyright 2022-2024 Riccardo Monica
 * 
 * This software is distributed under the 3-clause BSD license.
 * You should have received a copy of the 3-clause BSD license
 * along with this software. If not, see
 * <https://opensource.org/license/bsd-3-clause>
 */

#ifndef TRACKING_INTERFACE_H
#define TRACKING_INTERFACE_H

#include <memory>
#include <map>

#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace Tracking
{
  enum Parameters
  {
    PARAM_NONE                  = -1,

    // constrained filters
    PARAM_TRANSLATION_LIMIT     = 0,
    PARAM_ROTATION_LIMIT        = 1,
    PARAM_TRANSLATION_DYN_LIMIT = 2,
    PARAM_ROTATION_DYN_LIMIT    = 3,
    PARAM_ROTATION_CURV_LIMIT   = 4,

    // complementary filter, exp smoothing, double exp smoothing
    PARAM_ALPHA                 = 5,

    // particle filter & kalman filter
    PARAM_NUM_PARTICLES         = 6,
    PARAM_SIGMA_ROT             = 7, // update step
    PARAM_SIGMA_TRANSL          = 8, // update step
    PARAM_PSIGMA_ROT            = 9, // prediction step
    PARAM_PSIGMA_TRANSL         = 10, // prediction step
    PARAM_ISIGMA_ROT            = 11, // init step
    PARAM_ISIGMA_TRANSL         = 12, // init_step

    // wrapper kalman filter
    PARAM_W_SIGMA_ROT           = 13, // update step
    PARAM_W_SIGMA_TRANSL        = 14, // update step
    PARAM_W_PSIGMA_ROT          = 15, // prediction step
    PARAM_W_PSIGMA_TRANSL       = 16, // prediction step
    PARAM_W_ISIGMA_ROT          = 17, // init step
    PARAM_W_ISIGMA_TRANSL       = 18, // init_step

    PARAM_ROTATION_DYN2_LIMIT    = 19,
    PARAM_ROTATION_CURV2_LIMIT   = 20,
    PARAM_TRANSLATION_DYN2_LIMIT = 21,

    PARAM_MULT                   = 22,
  };

  enum State
  {
    STATE_SMALL_DIFF = 0,
    STATE_MEDIUM_DIFF = 1,
    STATE_REPOSITION = 2,
    STATE_INIT = 3
  };

  class ITracking
  {
    public:
    typedef std::shared_ptr<ITracking> Ptr;

    virtual ~ITracking() {}

    virtual void Update(const Eigen::Affine3f & m_world_to_motive_mat,
                        const Eigen::Affine3f & m_oculus_origin_to_oculus) = 0;

    virtual bool IsMotiveLost() const = 0;

    virtual void Reset() = 0;
    virtual void Reset(const Eigen::Affine3f & initial_world_to_oculus,
                       const Eigen::Affine3f & initial_oculus_origin_to_oculus) = 0;

    virtual Eigen::Affine3f GetEstimatedOculus() const = 0;

    virtual int GetPositionStatus() const = 0;

    virtual int GetRotationStatus() const = 0;

    virtual bool HasConverged() const {return false; }
    virtual void SetNotConverged() {}

    virtual bool SetParameter(const int parameter_id, const float value)
    {
      m_params[parameter_id] = value;
      return true;
    }

    virtual float GetParameter(const int parameter_id)
    {
      if (m_params.find(parameter_id) != m_params.end())
        return m_params[parameter_id];
      return -1.0f;
    }

    protected:
    std::map<int, float> m_params;
  };


}



#endif // TRACKING_INTERFACE_H
