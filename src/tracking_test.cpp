/*
 * Copyright 2022-2024 Riccardo Monica
 * 
 * This software is distributed under the 3-clause BSD license.
 * You should have received a copy of the 3-clause BSD license
 * along with this software. If not, see
 * <https://opensource.org/license/bsd-3-clause>
 */

#include "tracking_library.h"
#include "statistics_tracker.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <stdint.h>
#include <string>
#include <chrono>
#include <memory>

#include <Eigen/Dense>
#include <Eigen/StdVector>

typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > Vector3fVector;

typedef uint64_t uint64;
typedef uint32_t uint32;

bool ReadMatrix(std::istream & istr, Eigen::Affine3f & result)
{
  for (uint32 y = 0; y < 4; y++)
  {
    std::string line;
    std::getline(istr, line);
    if (!istr && y != 0)
      std::cout << "Unexpected end of file at row index " << y << std::endl;
    if (!istr)
      return false;

    std::istringstream linestream(line);
    for (uint32 x = 0; x < 4; x++)
    {
      float v;
      linestream >> v;
      if (!linestream)
      {
        std::cout << "Unexpected end of line at line: " << line << std::endl;
        return false;
      }
      result.matrix()(y, x) = v;
    }
  }

  std::string line;
  std::getline(istr, line);
  if (!istr)
    return false; // eof
  if (line.find_first_not_of("\n\r\t ") != std::string::npos)
  {
    std::cout << "Expected empty line at end of matrix, found " << line << std::endl;
    return false;
  }

  return true;
}

const float PI = std::acos(-1.0f);

int main(int argc, char ** argv)
{
  if (argc < 5)
  {
    std::cout << "Usage: tracking_test mode motive_data.txt oculus_data.txt error.txt [output_traj.txt]" << std::endl;
    std::exit(1);
  }

  const std::string mode_str = argv[1];
  const std::string motive_filename = argv[2];
  const std::string oculus_filename = argv[3];
  const std::string error_filename = argv[4];
  const std::string output_traj_filename = (argc > 5) ? argv[5] : "";

  std::cout << "Opening motive file: " << motive_filename << std::endl;
  std::ifstream mfile(motive_filename.c_str());
  if (!mfile)
  {
    std::cout << "Unable to open file: " << motive_filename << std::endl;
    std::exit(2);
  }

  std::cout << "Opening oculus file: " << oculus_filename << std::endl;
  std::ifstream ofile(oculus_filename.c_str());
  if (!ofile)
  {
    std::cout << "Unable to open file: " << oculus_filename << std::endl;
    std::exit(3);
  }

  std::shared_ptr<std::ofstream> otraj_file;
  if (!output_traj_filename.empty())
  {
    std::cout << "Writing output traj file: " << output_traj_filename << std::endl;
    otraj_file.reset(new std::ofstream(output_traj_filename));
    if (!*otraj_file)
    {
      std::cout << "Unable to create file: " << output_traj_filename << std::endl;
      std::exit(5);
    }
  }

  std::cout << "Writing error file: " << error_filename << std::endl;
  std::ofstream efile(error_filename.c_str());
  if (!efile)
  {
    std::cout << "Unable to create file: " << oculus_filename << std::endl;
    std::exit(4);
  }
  efile << "frame_count\t"
           "pos_err\t"
           "rot_err\t"
           "pos_diff\t"
           "rot_diff\t"
           "pos_vib\t"
           "rot_vib\t"
           "mot_lost\t"
           "ctime\n";

  int mode = Tracking::MODE_CONSTR_COMPLEMENTARY;
  bool is_wrapped = false;
  if (mode_str == "MODE_CONSTR_COMPLEMENTARY")
  {
    mode = Tracking::MODE_CONSTR_COMPLEMENTARY;
    is_wrapped = true;
  }
  else if (mode_str == "MODE_OCULUS_ORIGIN")
  {
    mode = Tracking::MODE_OCULUS_ORIGIN;
    is_wrapped = true;
  }
  else if (mode_str == "MODE_DIR_COMPLEMENTARY")
  {
    mode = Tracking::MODE_DIR_COMPLEMENTARY;
    is_wrapped = true;
  }
  else if (mode_str == "MODE_COMPLEMENTARY")
  {
    mode = Tracking::MODE_COMPLEMENTARY;
    is_wrapped = true;
  }
  else if (mode_str == "MODE_MOTIVE_FIRST")
  {
    mode = Tracking::MODE_MOTIVE_FIRST;
    is_wrapped = true;
  }
  else if (mode_str == "MODE_DOUBLE_EXP")
    mode = Tracking::MODE_DOUBLE_EXP;
  else if (mode_str == "MODE_EXP")
    mode = Tracking::MODE_EXP;
  else if (mode_str == "MODE_OCULUS_FIRST")
  {
    mode = Tracking::MODE_OCULUS_FIRST;
    is_wrapped = true;
  }
  else if (mode_str == "MODE_PARTICLE_FILTER")
    mode = Tracking::MODE_PARTICLE_FILTER;
  else if (mode_str == "MODE_KALMAN_FILTER")
    mode = Tracking::MODE_KALMAN_FILTER;
  else if (mode_str == "MODE_CONSTRAINED_KALMAN_FILTER")
    mode = Tracking::MODE_CONSTR_KALMAN_FILTER;
  else
  {
    std::cout << "Unknown mode " << mode_str << ", available modes:\n"
                                                "MODE_CONSTR_COMPLEMENTARY\n"
                                                "MODE_DIR_COMPLEMENTARY\n"
                                                "MODE_OCULUS_ORIGIN\n"
                                                "MODE_COMPLEMENTARY\n"
                                                "MODE_MOTIVE_FIRST\n"
                                                "MODE_DOUBLE_EXP\n"
                                                "MODE_EXP\n"
                                                "MODE_PARTICLE_FILTER\n"
                                                "MODE_KALMAN_FILTER\n"
                                                "MODE_CONSTRAINED_KALMAN_FILTER\n"
                                                "MODE_OCULUS_FIRST" << std::endl;
    std::exit(1);
  }

  if (!is_wrapped)
    TrackingSetMode(mode);
  else
    TrackingSetKalmanWrappedMode(mode);

  TrackingReset();

  Tracking::StatisticsTracker statistics_tracker;

  Eigen::Affine3f oculus_origin_to_oculus, world_to_motive;
  Eigen::Affine3f prev_world_to_oculus_out;
  uint64 frame_count = 0;
  while (ReadMatrix(ofile, oculus_origin_to_oculus) && ReadMatrix(mfile, world_to_motive))
  {
    std::cout << "--- FRAME " << frame_count << " ---" << std::endl;
    std::cout << "---Oculus---" << std::endl;
    std::cout << oculus_origin_to_oculus.matrix() << std::endl << std::endl;
    std::cout << "---Motive---" << std::endl;
    std::cout << world_to_motive.matrix() << std::endl << std::endl;

    float oculus_mat_floatv[16];
    float motive_mat_floatv[16];
    Tracking::Affine3fToFloatv(oculus_origin_to_oculus, oculus_mat_floatv);
    Tracking::Affine3fToFloatv(world_to_motive, motive_mat_floatv);

    auto start_update_time = std::chrono::system_clock::now();

    TrackingUpdate(motive_mat_floatv, oculus_mat_floatv);

    auto end_update_time = std::chrono::system_clock::now();
    double elapsed_seconds = std::chrono::duration<double>(end_update_time - start_update_time).count();

    float estimated_oculus_mat_floatv[16];
    TrackingGetEstimatedOculus(estimated_oculus_mat_floatv);
    Eigen::Affine3f world_to_oculus_out = Tracking::FloatvToAffine3f(estimated_oculus_mat_floatv);

    if (otraj_file)
    {
      (*otraj_file) << world_to_oculus_out.matrix() << std::endl << std::endl;
    }

    std::cout << "---OculusOrigin*Oculus---" << std::endl;
    std::cout << world_to_oculus_out.matrix() << std::endl << std::endl;

    std::string delta_dist_str = "-";
    std::string delta_rot_str = "-";

    if (frame_count != 0)
    {
      Eigen::Affine3f delta = prev_world_to_oculus_out.inverse() * world_to_oculus_out;
      const float angle_delta = std::abs(Eigen::AngleAxisf(delta.linear()).angle());
      const float position_delta = delta.translation().norm();
      delta_rot_str = std::to_string(angle_delta);
      delta_dist_str = std::to_string(position_delta);
    }
    prev_world_to_oculus_out = world_to_oculus_out;

    statistics_tracker.Update(world_to_motive, world_to_oculus_out);

    const float vibration_rad = statistics_tracker.GetVibrationRad();
    const float vibration_met = statistics_tracker.GetVibrationMet();

    if (TrackingIsMotiveLost())
    {
      std::cout << "Motive Lost!" << std::endl;
      efile << frame_count << "\t" << "-" << "\t" << "-" << "\t" << delta_dist_str << "\t" << delta_rot_str << "\t"
            << vibration_met << "\t" << vibration_rad << "\tYES\t" << elapsed_seconds << "\n";
    }
    else
    {
      Eigen::Affine3f error = world_to_oculus_out.inverse() * world_to_motive;
      std::cout << "---Error---" << std::endl;
      std::cout << error.matrix() << "\n" << std::endl;
      const float angle_error = std::abs(Eigen::AngleAxisf(error.linear()).angle());
      const float position_error = error.translation().norm();
      std::cout << "angle error: " << (angle_error * 180.0f / PI) << " position error: " << position_error << std::endl;
      efile << frame_count << "\t" << "\t" << position_error << "\t" << angle_error << "\t" << delta_dist_str
            << "\t" << delta_rot_str << "\t"
            << vibration_met << "\t" << vibration_rad << "\tNO\t" << elapsed_seconds  << "\n";
    }
    std::cout << (TrackingHasConverged() ? "CONVERGED" : "NOT CONVERGED") << std::endl;

    frame_count++;
  }

  return 0;
}
