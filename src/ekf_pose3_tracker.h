/*
 * Copyright 2022-2024 Dario Lodi Rizzini
 * 
 * This software is distributed under the 3-clause BSD license.
 * You should have received a copy of the 3-clause BSD license
 * along with this software. If not, see
 * <https://opensource.org/license/bsd-3-clause>
 */

#ifndef EKF_POSE3_TRACKER_H_
#define EKF_POSE3_TRACKER_H_

#if _MSC_VER // this is defined when compiling with Visual Studio
#define EXPORT_API __declspec(dllexport) // Visual Studio needs annotating exported functions with this
#else
#define EXPORT_API
#endif

#include <iostream>
#include <Eigen/Dense>
#include <manif/manif.h>

namespace ekfpt { 

    class EkfPose3Tracker {
    public:

        using Scalar = double;
        using Vector6 = Eigen::Matrix<Scalar, 6, 1>;
        using Matrix6 = Eigen::Matrix<Scalar, 6, 6>;
        using SE3 = manif::SE3d;
        using SE3Tangent = manif::SE3Tangentd;

        EXPORT_API EkfPose3Tracker();

        EXPORT_API virtual ~EkfPose3Tracker();

        EXPORT_API void initState(const SE3& stateMean, const Matrix6& stateCovar);

        EXPORT_API SE3 getMean() const;
        
        EXPORT_API Matrix6 getCovar() const;
        
        EXPORT_API void updatePrediction(const SE3& u, const Matrix6& noiseCovar);

        EXPORT_API void updatePrediction(const SE3Tangent& u, const Matrix6& noiseCovar);

        EXPORT_API void updateCorrection(const SE3& z, const Matrix6& noiseCovar);

        EXPORT_API void updateCorrectionWithLimit(const SE3& z, const Matrix6& noiseCovar,
                                       const float limit_distance, const float limit_angle); // meters, radians

    private:
        SE3 stateMean_;
        Matrix6 stateCovar_;
    };

} // end of namespace 

#endif
