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
#include <functional>
#include <set>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "tracking_library.h"
#include "tracking_interface.h"

#include "tracking_parser_utils.h"
#include "tracking_parser.h"

typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > Vector3fVector;
typedef std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f> > Affine3fVector;
typedef std::vector<std::string> StringVector;

typedef uint64_t uint64;
typedef std::set<uint64> Uint64Set;
typedef uint32_t uint32;
typedef int64_t int64;
typedef int32_t int32;
#define _1 (std::placeholders::_1)
#define _2 (std::placeholders::_2)

const float PI = std::acos(-1.0f);

int ModeStringToMode(const std::string & mode_str)
{
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

  return mode;
}

struct SubTrajectory
{
  Affine3fVector motive;
  Affine3fVector oculus;
  Affine3fVector exp_pose; // filter output

  uint64 hide_at_frame = 0;
  uint64 reveal_at_frame = 0;

  typedef std::shared_ptr<SubTrajectory> Ptr;
  typedef std::shared_ptr<const SubTrajectory> ConstPtr;
};

class SubTrajectoryListener
{
  public:
  struct Stats
  {
    uint64 frame_num = 0;
    float pos_err = 0.0f;
    float rot_err = 0.0f;
    float pos_diff = 0.0f;
    float rot_diff = 0.0f;
    float pos_vib = 0.0f;
    float rot_vib = 0.0f;
    float ctime = 0.0f;

    Stats operator+(const Stats & other) const
    {
      Stats result = *this;
      result.pos_err += other.pos_err;
      result.rot_err += other.rot_err;
      result.pos_diff += other.pos_diff;
      result.rot_diff += other.rot_diff;
      result.pos_vib += other.pos_vib;
      result.rot_vib += other.rot_vib;
      result.ctime += other.ctime;
      result.frame_num += other.frame_num;
      return result;
    }

    Stats operator+=(const Stats & other)
    {
      (*this) = (*this) + other;
      return *this;
    }

    Stats operator/(const float divisor) const
    {
      Stats result = *this;
      result.pos_err /= divisor;
      result.rot_err /= divisor;
      result.pos_diff /= divisor;
      result.rot_diff /= divisor;
      result.pos_vib /= divisor;
      result.rot_vib /= divisor;
      result.ctime /= divisor;
      return result;
    }
  };
  typedef std::vector<Stats> StatsVector;
  typedef std::vector<StatsVector> StatsVectorVector;

  struct SubTrajectoryInfo
  {
    uint64 hide_at_frame;
    uint64 reveal_at_frame;
    uint64 end_at_frame;

    explicit SubTrajectoryInfo(const SubTrajectory & traj)
    {
      hide_at_frame = traj.hide_at_frame;
      reveal_at_frame = traj.reveal_at_frame;
      end_at_frame = traj.motive.size();
    }
  };
  typedef std::vector<SubTrajectoryInfo> SubTrajectoryInfoVector;

  SubTrajectoryListener(const std::string &  mode, const int64 reveal_at_frame, const int64 total_frames,
                        const Uint64Set & save_traj_id, const std::string & save_traj_prefix)
  {
    m_mode = mode;

    m_save_traj_id = save_traj_id;
    m_save_traj_prefix = save_traj_prefix;

    m_reveal_at_frame = reveal_at_frame;
    m_total_frames = total_frames;
    m_stats.resize(m_total_frames);

    m_counter = 0;
  }

  void ApplyModeString(const std::string & mode_str)
  {
    try
    {
      std::istringstream iss(mode_str);
      std::string mode;
      iss >> mode;
      Tracking::Mode m = StringToMode(mode);

      std::vector<std::pair<int, float> > parameters;

      bool is_wrapped = false;
      std::string str;
      while (iss >> str)
      {
        if (str == "KW")
        {
          is_wrapped = true;
          continue;
        }

        Tracking::Parameters par = StringToParameter(str);
        float f;
        if (!(iss >> f))
          throw std::string("Value expected after parameter " + str);
        parameters.push_back(std::pair<int, float>(par, f));
      }

      if (!is_wrapped)
        TrackingSetMode(m);
      else
        TrackingSetKalmanWrappedMode(m);

      for (const std::pair<int, float> & p : parameters)
        TrackingSetParameter(p.first, p.second);
    }
    catch (std::string e)
    {
      std::cout << "Could not parse mode string: " << mode_str << "\n";
      std::cout << "Exception: " << e << std::endl;
      std::exit(1);
    }
  }

  void PrintStats(std::ostream & efile)
  {
    efile << "frame_count\t"
             "pos_err\t"
             "rot_err\t"
             "pos_diff\t"
             "rot_diff\t"
             "pos_vib\t"
             "rot_vib\t"
             "mot_lost\t"
             "ctime\t"
             "count\n";

    for (uint64 frame_i = 0; frame_i < m_stats.size(); frame_i++)
    {
      const uint64 frame_num = m_stats[frame_i].frame_num;
      const Stats & stats = m_stats[frame_i] / float(frame_num);
      efile << frame_i << "\t" << "\t" << stats.pos_err << "\t" << stats.rot_err << "\t" << stats.pos_diff
            << "\t" << stats.rot_diff << "\t"
            << stats.pos_vib << "\t" << stats.rot_vib << "\tNO\t" << stats.ctime << "\t" << frame_num << "\n";
    }
  }

  struct AverageErrorStats
  {
    float avg_pos_err;
    float avg_rot_err;
    uint64 num_frames;
  };

  AverageErrorStats GetAverageErrorStats()
  {
    AverageErrorStats aestats;
    aestats.avg_pos_err = 0.0f;
    aestats.avg_rot_err = 0.0f;
    aestats.num_frames = 0;
    for (uint64 frame_i = 0; frame_i < m_stats.size(); frame_i++)
    {
      if (frame_i < m_reveal_at_frame)
        continue;
      const uint64 frame_num = m_stats[frame_i].frame_num;
      const Stats & stats = m_stats[frame_i] / float(frame_num);
      aestats.avg_pos_err += stats.pos_err;
      aestats.avg_rot_err += stats.rot_err;
      aestats.num_frames += 1;
    }

    if (aestats.num_frames != 0)
    {
      aestats.avg_pos_err /= aestats.num_frames;
      aestats.avg_rot_err /= aestats.num_frames;
    }

    return aestats;
  }

  AverageErrorStats GetErrorByTrajStats(const uint64 traj_idx)
  {
    AverageErrorStats aestats;
    aestats.avg_pos_err = 0.0f;
    aestats.avg_rot_err = 0.0f;
    aestats.num_frames = 0;
    StatsVector my_stats = m_by_trajectory_stats[traj_idx];
    for (uint64 frame_i = 0; frame_i < my_stats.size(); frame_i++)
    {
      if (frame_i < m_reveal_at_frame)
        continue;
      const uint64 frame_num = my_stats[frame_i].frame_num;
      if (!frame_num)
        continue;
      const Stats & stats = my_stats[frame_i];
      aestats.avg_pos_err += stats.pos_err;
      aestats.avg_rot_err += stats.rot_err;
      aestats.num_frames += 1;
    }

    if (aestats.num_frames != 0)
    {
      aestats.avg_pos_err /= aestats.num_frames;
      aestats.avg_rot_err /= aestats.num_frames;
    }

    return aestats;
  }

  void PrintSubTrajectoryInfo(std::ostream & efile)
  {
    for (const SubTrajectoryInfo & sti : m_infos)
    {
      efile << sti.hide_at_frame << "\t" << sti.reveal_at_frame << "\t" << sti.end_at_frame << "\n";
    }
  }

  void AddAverageErrorStatsToDB(const std::string & filename, const std::string & extra_data_filename, const std::string & idx)
  {
    AverageErrorStats aestats = GetAverageErrorStats();

    std::map<std::string, std::string> db;
    {
      std::ifstream ifile(filename);
      std::string line;
      while (std::getline(ifile, line))
      {
        if (line.empty())
          continue;
        std::istringstream iline(line);
        std::string record_idx;
        iline >> record_idx;
        if (!iline || record_idx.empty())
          continue;
        db[record_idx] = line;
      }
    } // close ifile

    std::map<std::string, std::string> extra_data;
    if (extra_data_filename != "")
    {
      std::ifstream ifile(extra_data_filename);
      std::string line;
      while (std::getline(ifile, line))
      {
        if (line.empty())
          continue;
        std::istringstream iline(line);
        std::string record_idx;
        iline >> record_idx;
        if (!iline || record_idx.empty())
          continue;
        std::string v;
        StringVector vs;
        while (iline >> v)
          vs.push_back(v);
        std::string complete_line;
        for (StringVector::const_iterator iter = vs.begin(); iter != vs.end(); iter++)
          complete_line += "\t" + (*iter);
        extra_data[record_idx] = complete_line;
      }
    } // close ifile

    std::string new_line = idx + "\t" + std::to_string(aestats.avg_pos_err) + "\t" + std::to_string(aestats.avg_rot_err);
    if (extra_data.find(idx) != extra_data.end())
      new_line += extra_data[idx];
    if (db[idx] == new_line)
      return; // nothing to do
    db[idx] = new_line;

    {
      std::ofstream ofile(filename);
      for (const std::pair<std::string, std::string> & v : db)
        ofile << v.second << "\n";
      if (!ofile)
      {
        std::cout << "AddErrorStatsToDB: could not write file: " << filename << std::endl;
        exit(1);
      }
    } // close ofile
  }

  void AddErrorByTrajToDB(const std::string & filename, const std::string & idx)
  {
    std::map<std::pair<std::string, uint64>, std::string> db;
    {
      std::ifstream ifile(filename);
      std::string line;
      while (std::getline(ifile, line))
      {
        if (line.empty())
          continue;
        std::istringstream iline(line);
        std::pair<std::string, uint64> record_idx;
        std::string alg_idx;
        uint64 traj_idx;
        iline >> alg_idx >> traj_idx;
        if (!iline || alg_idx.empty())
          continue;
        record_idx.first = alg_idx;
        record_idx.second = traj_idx;
        db[record_idx] = line;
      }
    } // close ifile

    for (uint64 traj_idx = 0; traj_idx < m_by_trajectory_stats.size(); traj_idx++)
    {
      AverageErrorStats aestats = GetErrorByTrajStats(traj_idx);

      std::pair<std::string, uint64> record_idx(idx, traj_idx);

      std::string new_line = idx + "\t" + std::to_string(traj_idx) + "\t" +
          std::to_string(aestats.avg_pos_err) + "\t" + std::to_string(aestats.avg_rot_err);
      db[record_idx] = new_line;
    }

    {
      std::ofstream ofile(filename);
      for (const std::pair<std::pair<std::string, uint64>, std::string> & v : db)
        ofile << v.second << "\n";
      if (!ofile)
      {
        std::cout << "AddErrorByTrajToDB: could not write file: " << filename << std::endl;
        exit(1);
      }
    } // close ofile
  }

  void OnSubTrajectory(const SubTrajectory & sub_trajectory)
  {
    std::cout << "Sub trajectory " << m_counter << " size " << sub_trajectory.motive.size() << "=" <<
                 sub_trajectory.oculus.size() << " hide at " << sub_trajectory.hide_at_frame <<
                 " reveal at " << sub_trajectory.reveal_at_frame << std::endl;

    m_infos.push_back(SubTrajectoryInfo(sub_trajectory));

    const bool record_this_traj = (m_save_traj_id.find(m_counter) != m_save_traj_id.end());
    std::shared_ptr<std::ofstream> efile; // statistics
    std::shared_ptr<std::ofstream> tfile; // translation only
    std::shared_ptr<std::ofstream> tmfile; // motive translation only
    std::shared_ptr<std::ofstream> newposefile; // new pose translation only
    if (record_this_traj)
    {
      const std::string efilename = m_save_traj_prefix + std::to_string(m_counter) + "_errors.txt";
      const std::string metafilename = m_save_traj_prefix + std::to_string(m_counter) + "_meta.txt";
      const std::string tfilename = m_save_traj_prefix + std::to_string(m_counter) + "_translation.txt";
      const std::string tmfilename = m_save_traj_prefix + std::to_string(m_counter) + "_translation_motive.txt";
      const std::string newposefilename = m_save_traj_prefix + std::to_string(m_counter) + "_translation_exp_pose.txt";

      std::cout << "  This trajectory will be recorded in\n  " << efilename << "\n  " << tfilename
                << "\t  " << tmfilename << "\t  " << metafilename << std::endl;
      efile.reset(new std::ofstream(efilename));
      tfile.reset(new std::ofstream(tfilename));
      tmfile.reset(new std::ofstream(tmfilename));
      newposefile.reset(new std::ofstream(newposefilename));

      if (!*efile)
      {
        std::cout << "Could not write file " << efilename << std::endl;
        std::exit(1);
      }

      if (!*tfile)
      {
        std::cout << "Could not write file " << tfilename << std::endl;
        std::exit(1);
      }

      if (!*tmfile)
      {
        std::cout << "Could not write file " << tmfilename << std::endl;
        std::exit(1);
      }

      if (!*newposefile)
      {
        std::cout << "Could not write file " << newposefilename << std::endl;
        std::exit(1);
      }


      std::ofstream metafile(metafilename);
      metafile << sub_trajectory.hide_at_frame << "\t" << sub_trajectory.reveal_at_frame << "\t" << sub_trajectory.motive.size();
      if (!metafile)
      {
        std::cout << "Could not write file " << metafilename << std::endl;
        std::exit(1);
      }
    }

    if (efile)
      (*efile) << "frame_count\t" "pos_err\t" "rot_err\t" "pos_diff\t"
                  "rot_diff\t" "pos_vib\t" "rot_vib\t" "mot_lost\t" "ctime\n";

    TrackingSetMode(Tracking::MODE_KALMAN_FILTER);
    TrackingReset();

    Tracking::StatisticsTracker statistics_tracker;

    StatsVector this_trajectory_stats(m_total_frames);

    Eigen::Affine3f prev_world_to_oculus_out = Eigen::Affine3f::Identity();
    Eigen::Affine3f prev_world_to_motive = Eigen::Affine3f::Identity();
    Eigen::Affine3f prev_oculus_origin_to_oculus = Eigen::Affine3f::Identity();
    for (int64 frame_i = 0; frame_i < sub_trajectory.motive.size(); frame_i++)
    {
      const Eigen::Affine3f oculus_origin_to_oculus = sub_trajectory.oculus[frame_i];
      const Eigen::Affine3f world_to_motive = sub_trajectory.motive[frame_i];
      const Eigen::Affine3f exp_pose_from_file = sub_trajectory.exp_pose[frame_i];
      //std::cout << "--- FRAME " << frame_i << " ---" << std::endl;
      //std::cout << "---Oculus---" << std::endl;
      //std::cout << oculus_origin_to_oculus.matrix() << std::endl << std::endl;
      //std::cout << "---Motive---" << std::endl;
      //std::cout << world_to_motive.matrix() << std::endl << std::endl;

      if (frame_i == 0)
      {
        prev_world_to_oculus_out = world_to_motive;
        prev_world_to_motive = world_to_motive;
        prev_oculus_origin_to_oculus = oculus_origin_to_oculus;
      }

      float oculus_origin_to_oculus_floatv[16];
      float world_to_motive_floatv[16];
      float prev_world_to_oculus_out_floatv[16];
      float prev_oculus_origin_to_oculus_floatv[16];
      Tracking::Affine3fToFloatv(oculus_origin_to_oculus, oculus_origin_to_oculus_floatv);
      Tracking::Affine3fToFloatv(world_to_motive, world_to_motive_floatv);
      Tracking::Affine3fToFloatv(prev_world_to_oculus_out, prev_world_to_oculus_out_floatv);
      Tracking::Affine3fToFloatv(prev_oculus_origin_to_oculus, prev_oculus_origin_to_oculus_floatv);

      if (frame_i == sub_trajectory.hide_at_frame)
      {
        TrackingSetMode(Tracking::MODE_OCULUS_FIRST);
      }

      /*
      if (frame_i >= sub_trajectory.hide_at_frame && frame_i < sub_trajectory.reveal_at_frame)
      {
        Tracking::Affine3fToFloatv(last_visible_motive, world_to_motive_floatv);
      }
      else
        last_visible_motive = world_to_motive;
      */

      if (frame_i == sub_trajectory.reveal_at_frame)
      {
        ApplyModeString(m_mode);
        TrackingResetWithInit(prev_world_to_oculus_out_floatv, prev_oculus_origin_to_oculus_floatv);
      }

      auto start_update_time = std::chrono::system_clock::now();

      TrackingUpdate(world_to_motive_floatv, oculus_origin_to_oculus_floatv);

      auto end_update_time = std::chrono::system_clock::now();
      double elapsed_seconds = std::chrono::duration<double>(end_update_time - start_update_time).count();

      float estimated_oculus_mat_floatv[16];
      TrackingGetEstimatedOculus(estimated_oculus_mat_floatv);
      Eigen::Affine3f world_to_oculus_out = Tracking::FloatvToAffine3f(estimated_oculus_mat_floatv);

      //std::cout << "---OculusOrigin*Oculus---" << std::endl;
      //std::cout << world_to_oculus_out.matrix() << std::endl << std::endl;

      std::string delta_dist_str = "-";
      std::string delta_rot_str = "-";

      float angle_delta = 0.0f;
      float position_delta = 0.0f;

      if (frame_i != 0)
      {
        Eigen::Affine3f delta = prev_world_to_oculus_out.inverse() * world_to_oculus_out;
        angle_delta = std::abs(Eigen::AngleAxisf(delta.linear()).angle());
        position_delta = delta.translation().norm();
        delta_rot_str = std::to_string(angle_delta);
        delta_dist_str = std::to_string(position_delta);
      }
      prev_world_to_oculus_out = world_to_oculus_out;
      prev_oculus_origin_to_oculus = oculus_origin_to_oculus;

      statistics_tracker.Update(world_to_motive, world_to_oculus_out);

      const float vibration_rad = statistics_tracker.GetVibrationRad();
      const float vibration_met = statistics_tracker.GetVibrationMet();

      const bool lost = world_to_motive.matrix() == prev_world_to_motive.matrix();

      Eigen::Affine3f error = world_to_oculus_out.inverse() * world_to_motive;
      //std::cout << "---Error---" << std::endl;
      //std::cout << error.matrix() << "\n" << std::endl;
      const float angle_error = std::abs(Eigen::AngleAxisf(error.linear()).angle());
      const float position_error = error.translation().norm();
      //std::cout << "angle error: " << (angle_error * 180.0f / PI) << " position error: " << position_error << std::endl;
      if (efile)
        (*efile) << frame_i << "\t" << "\t" << position_error << "\t" << angle_error << "\t" << delta_dist_str
                 << "\t" << delta_rot_str << "\t"
                 << vibration_met << "\t" << vibration_rad << "\t" << (lost ? "YES" : "NO") << "\t" << elapsed_seconds  << "\n";
      if (tfile)
        (*tfile) << frame_i << "\t" << world_to_oculus_out.translation().x() << "\t"
                 << world_to_oculus_out.translation().y() << "\t"
                 << world_to_oculus_out.translation().z() << "\t" << (lost ? "LOST" : "OK") << "\n";
      if (tmfile)
        (*tmfile) << frame_i << "\t" << world_to_motive.translation().x() << "\t"
                  << world_to_motive.translation().y() << "\t"
                  << world_to_motive.translation().z() << "\t" << (lost ? "LOST" : "OK") << "\n";
      if (newposefile)
      {
        (*newposefile) << frame_i << "\t" << exp_pose_from_file.translation().x() << "\t"
                       << exp_pose_from_file.translation().y() << "\t"
                       << exp_pose_from_file.translation().z() << "\t" << (lost ? "LOST" : "OK") << "\n";
      }
      //std::cout << (TrackingHasConverged() ? "CONVERGED" : "NOT CONVERGED") << std::endl;

      if (world_to_motive.matrix() == prev_world_to_motive.matrix())
      {
        continue; // lost, do not save error
      }
      prev_world_to_motive = world_to_motive;

      const int64 stats_slot = m_reveal_at_frame - int64(sub_trajectory.reveal_at_frame) + frame_i;
      if (stats_slot >= 0 && stats_slot < m_total_frames)
      {
        Stats stats;
        stats.pos_err = position_error;
        stats.rot_err = angle_error;
        stats.pos_diff = position_delta;
        stats.rot_diff = angle_delta;
        stats.pos_vib = vibration_met;
        stats.rot_vib = vibration_rad;
        stats.ctime = elapsed_seconds;
        stats.frame_num = 1;
        m_stats[stats_slot] += stats;
        this_trajectory_stats[stats_slot] = stats;
      }

    }

    m_by_trajectory_stats.push_back(this_trajectory_stats);

    m_counter++;
  }


  private:
  uint64 m_counter;

  std::string m_mode;

  SubTrajectoryInfoVector m_infos;

  Uint64Set m_save_traj_id;
  std::string m_save_traj_prefix;

  StatsVector m_stats;
  StatsVectorVector m_by_trajectory_stats;
  int64 m_reveal_at_frame;
  int64 m_total_frames;
};

class ParserListener
{
  public:
  ParserListener(const std::set<Tracking::Mode> & only_modes,
                 const std::set<std::string> & only_mode_strings)
  {
    m_only_modes = only_modes;
    m_only_mode_strings = only_mode_strings;
  }

  void OnFrame(const uint64 frame)
  {
    //std::cout << "Frame: " << frame << std::endl;
  }

  void OnMotiveMatrix(const Eigen::Affine3f & m)
  {
    //std::cout << "Motive\n" << m.matrix() << std::endl;
    if (m_current_traj)
      m_current_traj->motive.push_back(m);
  }

  void OnExpPoseMatrix(const Eigen::Affine3f & m)
  {
    //std::cout << "Motive\n" << m.matrix() << std::endl;
    if (m_current_traj)
      m_current_traj->exp_pose.push_back(m);
  }

  void OnOculusMatrix(const Eigen::Affine3f & m)
  {
    //std::cout << "Oculus\n" << m.matrix() << std::endl;
    if (m_current_traj)
      m_current_traj->oculus.push_back(m);
  }

  void OnReposition()
  {
    if (m_current_traj)
    {
      std::cout << "Reposition " << std::endl;
      m_current_traj.reset();
    }
  }

  void OnAborted(const std::string & reason)
  {
    if (m_current_traj)
    {
      std::cout << "Aborted " << reason << std::endl;
      m_current_traj.reset();
    }
  }

  void OnModeChanged(const TrackingParser::Config & config)
  {
    //std::cout << "New config: " << ModeToString(config.mode) << std::endl;
  }

  bool IsAllowedMode(const TrackingParser::Config & config) const
  {
    std::string mode_string = ModeToString(config.mode);
    if (config.kalman_wrapped)
      mode_string += "KW";
    for (TrackingParser::ConfigParameterPair p : config.parameters)
      mode_string += ParameterToString(p.first) + std::to_string(p.second.value);

    bool is_allowed = m_only_modes.empty() ||
        (m_only_modes.find(config.mode) != m_only_modes.end());
    is_allowed = is_allowed && (m_only_mode_strings.empty() ||
                      m_only_mode_strings.find(mode_string) != m_only_mode_strings.end());

    return is_allowed;
  }

  void OnModePreChanged(const TrackingParser::Config & config)
  {
    //std::cout << "New pre-config: " << ModeToString(config.mode) << std::endl;
    const bool is_allowed_mode = IsAllowedMode(config);

    if (m_current_traj)
    {
      sub_trajectory_ls(*m_current_traj);
    }

    if (is_allowed_mode)
    {
      m_current_traj.reset(new SubTrajectory);
    }

    if (!is_allowed_mode)
    {
      m_current_traj.reset();
    }
  }

  void OnStatusChanged(const std::string & status, const bool visible)
  {
    //std::cout << status << " is visible " << (visible ? "YES" : "NO") << std::endl;
    if (status == "MOTIVE_HIDDEN")
    {
      if (m_current_traj)
        m_current_traj->hide_at_frame = m_current_traj->motive.size();
    }

    if (status == "MOTIVE_VISIBLE")
    {
      if (m_current_traj)
        m_current_traj->reveal_at_frame = m_current_traj->motive.size();
    }
  }

  void SetSubTrajectoryListener(std::function<void(const SubTrajectory &)> f)
  {
    sub_trajectory_ls = f;
  }

  private:
  std::set<Tracking::Mode> m_only_modes;
  std::set<std::string> m_only_mode_strings;

  std::function<void(const SubTrajectory &)> sub_trajectory_ls;

  SubTrajectory::Ptr m_current_traj;
};

struct FileGroup
{
  std::set<Tracking::Mode> only_modes;
  std::set<std::string> only_mode_strings;
  StringVector filenames;
};
typedef std::vector<FileGroup> FileGroupVector;

int main(int argc, char ** argv)
{
  if (argc < 6)
  {
    std::cout << "Usage: tracking_test mode_with_params reveal_at_frame end_at_frame {filenames} [{-s selected_modes}] "
                 "[-o output_file] [-sti sub_traj_info_file] [{-st save_traj_id{,save_traj_id}}] [{-sto st_file_prefix}]"
                 "[-calib calibration_file.matrix] "
                 "[-error_stats_file db_file.txt db_key [-error_stats_extra extra_file.txt] [-error_by_traj_file db_file.txt]]"
                 << std::endl;
    std::exit(1);
  }

  const std::string mode_str = argv[1];
  const std::string reveal_at_frame_str = argv[2];
  const std::string end_at_frame_str = argv[3];
  const int64 reveal_at_frame = std::stoul(reveal_at_frame_str);
  const int64 end_at_frame = std::stoul(end_at_frame_str);
  std::string ofilename;
  std::string stofilenameprefix;
  std::string stifilename;
  std::string calibfilename;
  std::string error_stats_filename;
  std::string error_by_trajectory_stats_filename;
  std::string error_stats_extra_filename;
  std::string error_stats_db_key;

  FileGroupVector groups;
  std::set<uint64> save_traj_id;

  {
    const StringVector params(argv + 4, argv + argc);
    FileGroup group;
    for (uint64 pi = 0; pi < params.size(); pi++)
    {
      if (params[pi] == "-newgroup")
      {
        if (!group.filenames.empty())
          groups.push_back(group);
        group = FileGroup();

        continue;
      }
      if (params[pi] == "-s")
      {
        pi++;
        if (pi >= params.size())
        {
          std::cout << "mode expected after -s" << std::endl;
          std::exit(1);
        }

        group.only_modes.insert(StringToMode(params[pi]));
        continue;
      }
      if (params[pi] == "-ss")
      {
        pi++;
        if (pi >= params.size())
        {
          std::cout << "mode string expected after -ss" << std::endl;
          std::exit(1);
        }

        group.only_mode_strings.insert(std::string(params[pi]));
        continue;
      }
      if (params[pi] == "-o")
      {
        pi++;
        if (pi >= params.size())
        {
          std::cout << "filename expected after -o" << std::endl;
          std::exit(1);
        }
        ofilename = params[pi];

        continue;
      }
      if (params[pi] == "-sto")
      {
        pi++;
        if (pi >= params.size())
        {
          std::cout << "filename prefix expected after -sto" << std::endl;
          std::exit(1);
        }
        stofilenameprefix = params[pi];

        continue;
      }
      if (params[pi] == "-sti")
      {
        pi++;
        if (pi >= params.size())
        {
          std::cout << "filename expected after -sti" << std::endl;
          std::exit(1);
        }
        stifilename = params[pi];

        continue;
      }
      if (params[pi] == "-st")
      {
        pi++;
        if (pi >= params.size())
        {
          std::cout << "traj_id(s) expected after -st" << std::endl;
          std::exit(1);
        }
        std::istringstream traj_id_ss(params[pi]);
        std::string traj_id_s;
        while (std::getline(traj_id_ss, traj_id_s, ','))
        {
          const uint64 id = std::stoul(traj_id_s);
          save_traj_id.insert(id);
        }

        continue;
      }
      if (params[pi] == "-error_stats_file")
      {
        pi++;
        if (pi >= params.size())
        {
          std::cout << "stats_file.txt expected after -error_stats_file" << std::endl;
          std::exit(1);
        }
        error_stats_filename = params[pi];
        pi++;
        if (pi >= params.size())
        {
          std::cout << "db_key expected after -error_stats_file [filename]" << std::endl;
          std::exit(1);
        }
        error_stats_db_key = params[pi];
        continue;
      }
      if (params[pi] == "-error_stats_extra")
      {
        pi++;
        if (pi >= params.size())
        {
          std::cout << "extra_stats_file.txt expected after -error_stats_extra" << std::endl;
          std::exit(1);
        }
        error_stats_extra_filename = params[pi];
        continue;
      }
      if (params[pi] == "-error_by_traj_file")
      {
        pi++;
        if (pi >= params.size())
        {
          std::cout << "error_by_traj_file.txt expected after -error_by_traj_file" << std::endl;
          std::exit(1);
        }
        error_by_trajectory_stats_filename = params[pi];
        continue;
      }
      if (params[pi] == "-calib")
      {
        pi++;
        if (pi >= params.size())
        {
          std::cout << "filename.matrix expected after -calib" << std::endl;
          std::exit(1);
        }
        calibfilename = params[pi];
        continue;
      }

      group.filenames.push_back(params[pi]);
    }

    if (!group.filenames.empty())
      groups.push_back(group);
  }

  std::cout << "Mode is \"" << mode_str << "\"" << std::endl;

  if (!save_traj_id.empty())
  {
    std::cout << "Saving trajectory ids: ";
    for (const uint64 id : save_traj_id)
      std::cout << id << " ";
    std::cout << std::endl;
    std::cout << "Saving prefix is " << stofilenameprefix << std::endl;
  }

  std::cout << "Loaded " << groups.size() << " groups." << std::endl;
  for (const FileGroup & grp : groups)
  {
    std::cout << "FILES:\n";
    for (const std::string & fn : grp.filenames)
      std::cout << "  " << fn << std::endl;

    std::cout << "ALLOWED MODES\n";
    for (const Tracking::Mode m : grp.only_modes)
      std::cout << "  " << ModeToString(m) << std::endl;
    for (const std::string s : grp.only_mode_strings)
      std::cout << "  " << s << std::endl;
    std::cout << std::endl;
  }

  std::shared_ptr<std::ofstream> ofile;
  if (!ofilename.empty())
  {
    std::cout << "Saving to file " << ofilename << std::endl;
    ofile.reset(new std::ofstream(ofilename));
    if (!*ofile)
    {
      std::cout << "Could not open output file " << ofilename << std::endl;
      std::exit(1);
    }
  }

  SubTrajectoryListener sub_trajectory_listener(mode_str, reveal_at_frame, end_at_frame,
                                                save_traj_id, stofilenameprefix);

  for (const FileGroup & group : groups)
    for (const std::string & filename : group.filenames)
    {
      std::cout << "Opening file: " << filename << std::endl;
      std::ifstream ifile(filename.c_str());
      if (!ifile)
      {
        std::cout << "Could not open file: " << filename << std::endl;
        std::exit(2);
      }

      ParserListener pl(group.only_modes, group.only_mode_strings);
      pl.SetSubTrajectoryListener(std::bind(&SubTrajectoryListener::OnSubTrajectory, &sub_trajectory_listener, _1));
      TrackingParser parser;
      parser.SetFrameListener(std::bind(&ParserListener::OnFrame, &pl, _1));
      parser.SetAbortedListener(std::bind(&ParserListener::OnAborted, &pl, _1));
      parser.SetModeChangedListener(std::bind(&ParserListener::OnModeChanged, &pl, _1));
      parser.SetMotiveListener(std::bind(&ParserListener::OnMotiveMatrix, &pl, _1));
      parser.SetExpPoseListener(std::bind(&ParserListener::OnExpPoseMatrix, &pl, _1));
      parser.SetOculusListener(std::bind(&ParserListener::OnOculusMatrix, &pl, _1));
      parser.SetRepositionListener(std::bind(&ParserListener::OnReposition, &pl));
      parser.SetStatusChangedListener(std::bind(&ParserListener::OnStatusChanged, &pl, _1, _2));
      parser.SetModePreChangedListener(std::bind(&ParserListener::OnModePreChanged, &pl, _1));
      parser.SetDebugLogListener([](const std::string &) {}); // disable parser debug log

      if (!calibfilename.empty())
      {
        std::cout << "Opening calibration file: " << calibfilename << std::endl;
        std::ifstream cfile(calibfilename);
        if (!cfile)
        {
          std::cout << "Could not open calibration file: " << calibfilename << std::endl;
          std::exit(4);
        }
        Eigen::Affine3f calib_mat = parser.ParseMatrix(cfile);
        parser.SetCalibrationMatrix(calib_mat);
      }

      try
      {
        parser.Parse(ifile);
      }
      catch (std::string str)
      {
        std::cout << "Parsing error in file : " << str << std::endl;
        std::exit(3);
      }
    }

  if (!stifilename.empty())
  {
    std::ofstream sub_trajectory_file(stifilename);
    sub_trajectory_listener.PrintSubTrajectoryInfo(sub_trajectory_file);
  }

  if (!ofile)
    std::cout << "--- PRINT STATS ---" << std::endl;
  {
    std::ostream & ofilestream = (ofile ? (*ofile) : std::cout);
    sub_trajectory_listener.PrintStats(ofilestream);
  }

  if (!error_stats_filename.empty())
  {
    sub_trajectory_listener.AddAverageErrorStatsToDB(error_stats_filename, error_stats_extra_filename, error_stats_db_key);
  }

  if (!error_by_trajectory_stats_filename.empty())
  {
    sub_trajectory_listener.AddErrorByTrajToDB(error_by_trajectory_stats_filename, error_stats_db_key);
  }

  return 0;
}
