/*
 * Copyright 2022-2024 Dario Lodi Rizzini
 * 
 * This software is distributed under the 3-clause BSD license.
 * You should have received a copy of the 3-clause BSD license
 * along with this software. If not, see
 * <https://opensource.org/license/bsd-3-clause>
 */

#include "ekf_pose3_tracker.h"

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

namespace ekfpt {

    EXPORT_API EkfPose3Tracker::EkfPose3Tracker() : stateMean_(), stateCovar_() {
    }

    EXPORT_API EkfPose3Tracker::~EkfPose3Tracker() {
    }

    EXPORT_API void EkfPose3Tracker::initState(const SE3& stateMean, const Matrix6& stateCovar) {
        stateMean_ = stateMean;
        stateCovar_ = stateCovar;
    }

    EXPORT_API EkfPose3Tracker::SE3 EkfPose3Tracker::getMean() const {
        return stateMean_;
    }

    EXPORT_API EkfPose3Tracker::Matrix6 EkfPose3Tracker::getCovar() const {
        return stateCovar_;
    }
    
    EXPORT_API void EkfPose3Tracker::updatePrediction(const SE3& u, const Matrix6& noiseCovar) {
        SE3Tangent ut = u.log();
        updatePrediction(ut, noiseCovar);
    }

    EXPORT_API void EkfPose3Tracker::updatePrediction(const SE3Tangent& u, const Matrix6& noiseCovar) {
        SE3::Jacobian jacobX, jacobU;
        //std::cout << "u " << u.coeffs().transpose() << ", u.exp()\n" << u.exp() << std::endl;
        //stateMean_ = stateMean_.plus(u, jacobX, jacobU);
        stateMean_ = stateMean_.compose(u.exp(),  jacobX, jacobU);
        stateCovar_ = jacobX * stateCovar_ * jacobX.transpose() + jacobU * noiseCovar * jacobU.transpose();
        //std::cout << "jacobX\n" << jacobX << "\njacobU\n" << jacobU << std::endl;
    }
    
    EXPORT_API void EkfPose3Tracker::updateCorrection(const SE3& z, const Matrix6& noiseCovar) {
        SE3Tangent dzTang;
        Vector6 dz;
        Matrix6 K, H;
        SE3Tangent dx;
        SE3::Jacobian jacobInv, jacobComX, jacobComZ, jacobLog;
        
        //SE3Tangent dz = z.minus(stateMean_);
        //dz = z.minus(stateMean_, jacobZ, jacobX).coeffs();
        dzTang = (stateMean_.inverse(jacobInv).compose(z, jacobComX, jacobComZ)).log(jacobLog);
        dz = dzTang.coeffs();
//        std::cout << "jacobInv\n" << jacobInv << "\njacobComX\n" << jacobComX << "\njacobComZ\n" << jacobComZ 
//                << "\njacobLog\n" << jacobLog 
//                << "\nH = jacobLog * njacobComX * jacobInv\n" << (jacobLog * jacobComX * jacobInv) << std::endl;
        H = Matrix6::Identity();
        //H = jacobLog * jacobComX * jacobInv;
        K = stateCovar_ * H.transpose() * (H * stateCovar_ * H.transpose() + noiseCovar).inverse();
        dx = K * dz;
        //stateMean_ = stateMean_.rplus(dx);    // stateMean_ + (K * dz) = stateMean_ * exp( (K * dz).wedge() )
        stateMean_ = stateMean_.compose(dx.exp());
        stateCovar_ = (Matrix6::Identity() - K) * stateCovar_;
    }

    EXPORT_API void EkfPose3Tracker::updateCorrectionWithLimit(const SE3& z, const Matrix6& noiseCovar,
                                                    const float limit_distance, const float limit_angle)
    {
      SE3Tangent dzTang;
      Vector6 dz;
      Matrix6 K, H;
      SE3Tangent dx;
      SE3::Jacobian jacobInv, jacobComX, jacobComZ, jacobLog;

      //SE3Tangent dz = z.minus(stateMean_);
      //dz = z.minus(stateMean_, jacobZ, jacobX).coeffs();
      dzTang = (stateMean_.inverse(jacobInv).compose(z, jacobComX, jacobComZ)).log(jacobLog);
      dz = dzTang.coeffs();
//        std::cout << "jacobInv\n" << jacobInv << "\njacobComX\n" << jacobComX << "\njacobComZ\n" << jacobComZ
//                << "\njacobLog\n" << jacobLog
//                << "\nH = jacobLog * njacobComX * jacobInv\n" << (jacobLog * jacobComX * jacobInv) << std::endl;
      H = Matrix6::Identity();
      //H = jacobLog * jacobComX * jacobInv;
      K = stateCovar_ * H.transpose() * (H * stateCovar_ * H.transpose() + noiseCovar).inverse();
      dx = K * dz;

      SE3 dx_exp = dx.exp();
      const Eigen::Affine3f dx_exp_eigen = SE3ToEigen(dx_exp);
      const float distance_in_meters = dx_exp_eigen.translation().norm();
      const float distance_in_radians = std::abs(Eigen::AngleAxisf(dx_exp_eigen.linear()).angle());
      const float num_frames = std::max(limit_distance < distance_in_meters ? distance_in_meters / limit_distance : 1.0f,
                                        limit_angle < distance_in_radians ? distance_in_radians / limit_angle : 1.0f);
      Matrix6 Kreduced = K / num_frames;
      SE3Tangent dxreduced = Kreduced * dz;

      //stateMean_ = stateMean_.rplus(dx);    // stateMean_ + (K * dz) = stateMean_ * exp( (K * dz).wedge() )
      stateMean_ = stateMean_.compose(dxreduced.exp());
      //stateCovar_ = (Matrix6::Identity() - K) * stateCovar_;
      // Joseph form for non-optimal Kalman gain
      stateCovar_ = (Matrix6::Identity() - Kreduced) * stateCovar_ * (Matrix6::Identity() - Kreduced).transpose() +
                    Kreduced * noiseCovar * Kreduced.transpose();
    }

}
