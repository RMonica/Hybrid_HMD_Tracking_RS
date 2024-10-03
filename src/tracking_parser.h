/*
 * Copyright 2022-2024 Riccardo Monica
 * 
 * This software is distributed under the 3-clause BSD license.
 * You should have received a copy of the 3-clause BSD license
 * along with this software. If not, see
 * <https://opensource.org/license/bsd-3-clause>
 */

#ifndef TRACKING_PARSER_H
#define TRACKING_PARSER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdint.h>
#include <map>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "tracking_library.h"
#include "tracking_interface.h"

#include "tracking_parser_utils.h"

class TrackingParser
{
  public:
  typedef uint64_t uint64;
  typedef std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f> > Affine3fVector;
  typedef std::vector<std::string> StringVector;

  struct ConfigParameter
  {
    Tracking::Parameters parameter;
    float value;

    ConfigParameter(const Tracking::Parameters parameter, const float value)
    {
      this->parameter = parameter;
      this->value = value;
    }

    bool operator<(const ConfigParameter & other) const
    {
      if (int(parameter) < int(other.parameter))
        return true;
      if (int(parameter) > int(other.parameter))
        return false;

      if (value < other.value)
        return true;
      if (value > other.value)
        return false;

      return false;
    }

    bool operator>(const ConfigParameter & other) const
    {
      return !(*this < other) && !(*this == other);
    }

    bool operator==(const ConfigParameter & other) const
    {
      return this->parameter == other.parameter && this->value == other.value;
    }
  };

  typedef std::map<Tracking::Parameters, ConfigParameter> ConfigParameterMap;
  typedef std::pair<Tracking::Parameters, ConfigParameter> ConfigParameterPair;

  struct Config
  {
    Tracking::Mode mode;
    bool kalman_wrapped;
    std::string metadata;
    Tracking::Parameters evaluated_parameter;
    ConfigParameterMap parameters;

    Config(const Tracking::Mode mode, const bool kalman_wrapped)
    {
      this->mode = mode;
      this->kalman_wrapped = kalman_wrapped;
      this->evaluated_parameter = Tracking::PARAM_NONE;
    }

    Config WithParameter(const Tracking::Parameters parameter, const float value)
    {
      Config new_config = *this;
      new_config.parameters.insert(ConfigParameterPair(parameter, ConfigParameter(parameter, value)));
      return new_config;
    }

    bool operator<(const Config & other) const
    {
      if (int(mode) < int(other.mode))
        return true;
      if (int(mode) > int(other.mode))
        return false;

      if (int(evaluated_parameter) < int(other.evaluated_parameter))
        return true;
      if (int(evaluated_parameter) > int(other.evaluated_parameter))
        return false;

      // if we are here, same number of parameters
      ConfigParameterMap::const_iterator i1 = parameters.begin();
      ConfigParameterMap::const_iterator i2 = other.parameters.begin();

      for (; i1 != parameters.end() && i2 != other.parameters.end(); i1++, i2++)
      {
        if (*i1 < *i2)
          return true;
        if (*i1 > *i2)
          return false;
      }

      if (i1 == parameters.end() && i2 != other.parameters.end())
        return true;

      return false;
    }

    bool operator>(const Config & other) const
    {
      return !(*this < other) && !(*this == other);
    }

    bool operator==(const Config & other) const
    {
      return this->mode == other.mode && this->parameters == other.parameters;
    }
  };

  typedef std::shared_ptr<Config> ConfigPtr;

  struct ConfigResult
  {
    uint64 detected = 0;
    uint64 total    = 0;
    Config config   = Config(Tracking::MODE_OCULUS_ORIGIN, false);
  };

  typedef std::map<Config, ConfigResult> ConfigResultMap;
  typedef std::pair<Config, ConfigResult> ConfigResultPair;

  TrackingParser()
  {
    m_frame_num = 0;

    m_debug_logger_ls = [](const std::string & debug_log) {std::cout << debug_log << std::flush; };
  }

  Eigen::Affine3f ParseMatrix(std::istream & ifile)
  {
    std::string line;
    uint64 lines = 0;
    Eigen::Affine3f result;

    while (std::getline(ifile, line) && lines < 4)
    {
      if (line.empty())
        continue;

      std::istringstream line_str(line);
      for (uint64 i = 0; i < 4; i++)
      {
        float v;
        line_str >> v;
        if (!line_str)
          throw std::string("Unexpected EOL while parsing matrix at line: " + line);

        if (lines >= 3)
          continue; // affine: there is no last line

        result.matrix()(lines, i) = v;
      }
      lines++;
    }

    if (lines != 4)
      throw std::string("Unexpected EOF while parsing matrix at line: " + line);

    return result;
  }

  void Parse(std::istream & ifile)
  {
    m_frame_num = 0;

    ConfigResultMap config_map;
    Config current_config(Tracking::MODE_OCULUS_ORIGIN, false);
    Config last_visible_config = current_config;
    bool is_repositioned = false;
    bool is_current_state_visible = false;
    bool already_signalled = true;

    std::string line;
    while (std::getline(ifile, line))
    {
      std::istringstream linestream(line);

      std::string cmd;
      linestream >> cmd;

      if (!linestream)
        continue; // empty line

      if (cmd == "CONFIG" || cmd == "PRE_CONFIG")
      {
        //std::cout << m_frame_num << ": " << line << std::endl;
        std::string mode_str;
        linestream >> mode_str;
        Tracking::Mode mode = StringToMode(mode_str);
        std::string kalman_wrapped_string;
        linestream >> kalman_wrapped_string;
        bool kalman_wrapped = (kalman_wrapped_string == "KW");

        Config new_config(mode, kalman_wrapped);

        uint64 num_parameters;
        linestream >> num_parameters;
        if (!linestream)
          throw std::string("Unexpected EOL at line " + line);

        for (uint64 i = 0; i < num_parameters; i++)
        {
          std::string par_str;
          linestream >> par_str;
          Tracking::Parameters par = StringToParameter(par_str);
          float value;
          linestream >> value;
          new_config = new_config.WithParameter(par, value);
        }

        if (!linestream)
          throw std::string("Unexpected EOL at line " + line);

        while (linestream)
        {
          std::string cmd_extra;
          linestream >> cmd_extra;
          if (cmd_extra == "METADATA")
          {
            std::string metadata;
            linestream >> metadata;

            if (!linestream)
              throw std::string("METADATA: unexpected EOL at line " + line);


            new_config.metadata = metadata;

            try
            {
              new_config.evaluated_parameter = StringToParameter(metadata);
            }
            catch (std::string str)
            {
              //std::cout << "Warning: " << str << std::endl;
            }
          }
        }

        if (cmd == "PRE_CONFIG")
        {
          m_tracking_mode_pre_changed_ls(new_config);
        }
        else // cmd == "CONFIG"
        {
          m_tracking_mode_changed_ls(new_config);

          config_map[new_config].config = new_config;
          current_config = new_config;

          if (is_current_state_visible)
          {
            last_visible_config = current_config;
            config_map[new_config].total++;
            already_signalled = false;
          }

          is_repositioned = false;
        }
      }
      else if (cmd == "ABORTED")
      {
        std::string reason;
        linestream >> reason;
        m_aborted_ls(reason);
      }
      else if (cmd == "REPOSITION")
      {
//        if (!is_repositioned)
//          std::cout << m_frame_num << ": " << line << std::endl;

        m_reposition_ls();

        is_repositioned = true;
      }
      else if (cmd == "USERSIGNAL")
      {
//        std::cout << m_frame_num << ": " << line << std::endl;
        std::string signal;
        linestream >> signal;

        if (!is_repositioned && !already_signalled)
        {
          if (signal == "DETECTED_PRESSED" || signal == "MAYBE_PRESSED")
          {
            config_map[last_visible_config].detected++;
          }
          already_signalled = true;
        }
      }
      else if (cmd == "FRAME")
      {
        m_frame_num++;
        uint64 frame_n;
        linestream >> frame_n;
        m_frame_ls(frame_n);
      }
      else if (cmd == "STATE")
      {
        std::string state;
        linestream >> state;

        if (state == "MOTIVE_VISIBLE" || state == "MOTIVE_VISIBLE_END")
          is_current_state_visible = true;
        else
          is_current_state_visible = false;

        m_status_changed_ls(state, is_current_state_visible);
      }
      else if (cmd == "MOTIVE")
      {
        Eigen::Affine3f mat = ParseMatrix(ifile);
        mat = mat * m_calibration_matrix;
        m_motive_mats.push_back(mat);
        m_motive_mat_ls(mat);
      }
      else if (cmd == "OCULUS_OTO")
      {
        Eigen::Affine3f mat = ParseMatrix(ifile);
        m_oculus_mats.push_back(mat);
        m_oculus_mat_ls(mat);
      }
      else if (cmd == "OCULUS_NEW")
      {
        Eigen::Affine3f mat = ParseMatrix(ifile);
        m_exp_pose_ls(mat);
      }
      else if (cmd == "REGION")
      {
        StringVector region_data;
        while (linestream)
        {
          std::string region;
          linestream >> region;
          if (linestream)
            region_data.push_back(region);
        }
        const std::string r = region_data.empty() ? std::string("") : region_data[0];
        m_region_ls(r, region_data);
      }
    }

    for (const ConfigResultPair & conf : config_map)
    {
      m_debug_logger_ls((std::stringstream() << "Mode: " << ModeToString(conf.first.mode)).str());
      Tracking::Parameters evaluated_parameter = conf.first.evaluated_parameter;
      if (evaluated_parameter != Tracking::PARAM_NONE)
      {
        const ConfigParameterMap & parameters = conf.first.parameters;
        m_debug_logger_ls((std::stringstream() << " " << ParameterToString(evaluated_parameter) << "=").str());
        if (parameters.find(evaluated_parameter) != parameters.end())
          m_debug_logger_ls((std::stringstream() << parameters.find(evaluated_parameter)->second.value).str());
        else
          m_debug_logger_ls((std::stringstream() << "NOTFOUND").str());
      }
//      for (const ConfigParameterPair & par : conf.first.parameters)
//      {
//        std::cout << " " << ParameterToString(par.second.parameter) << "=" << par.second.value;
//      }
      m_debug_logger_ls((std::stringstream() << " " << conf.second.detected << "/" << conf.second.total).str());
      m_debug_logger_ls((std::stringstream() << "\n").str());
    }

    m_config_result_map = config_map;
  }

  ConfigResultMap GetConfigResultMap() const
  {
    return m_config_result_map;
  }

  Affine3fVector GetMotiveMats() const
  {
    return m_motive_mats;
  }

  Affine3fVector GetOculusMats() const
  {
    return m_oculus_mats;
  }

  void SetCalibrationMatrix(const Eigen::Affine3f & cm)
  {
    m_calibration_matrix = cm;
  }

  // LISTENERS
  void SetFrameListener(const std::function<void(const uint64)> f) {m_frame_ls = f; }
  void SetMotiveListener(const std::function<void(const Eigen::Affine3f &)> f) {m_motive_mat_ls = f; }
  void SetOculusListener(const std::function<void(const Eigen::Affine3f &)> f) {m_oculus_mat_ls = f; }
  void SetExpPoseListener(const std::function<void(const Eigen::Affine3f &)> f) {m_exp_pose_ls = f; }
  void SetRepositionListener(const std::function<void()> f) {m_reposition_ls = f; }
  void SetModeChangedListener(const std::function<void(const Config & config)> f) {m_tracking_mode_changed_ls = f; }
  void SetModePreChangedListener(const std::function<void(const Config & config)> f) {m_tracking_mode_pre_changed_ls = f; }
  void SetStatusChangedListener(const std::function<void(const std::string & state, const bool visible)> f) {m_status_changed_ls = f; }
  void SetAbortedListener(const std::function<void(const std::string & reason)> f) {m_aborted_ls = f; }
  void SetRegionListener(const std::function<void(const std::string & region, const StringVector & region_data)> f) {m_region_ls = f; }

  void SetDebugLogListener(const std::function<void(const std::string & debug_log)> f) {m_debug_logger_ls = f; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  private:
  uint64 m_frame_num;

  ConfigResultMap m_config_result_map;

  Affine3fVector m_motive_mats;
  Affine3fVector m_oculus_mats;

  Eigen::Affine3f m_calibration_matrix = Eigen::Affine3f::Identity();

  // listeners
  std::function<void(const uint64)> m_frame_ls = [](const uint64) {};
  std::function<void(const Eigen::Affine3f &)> m_motive_mat_ls = [](const Eigen::Affine3f &) {};
  std::function<void(const Eigen::Affine3f &)> m_oculus_mat_ls = [](const Eigen::Affine3f &) {};
  std::function<void(const Eigen::Affine3f &)> m_exp_pose_ls = [](const Eigen::Affine3f &) {};
  std::function<void()> m_reposition_ls = []() {};
  std::function<void(const Config & config)> m_tracking_mode_changed_ls = [](const Config &) {};
  std::function<void(const Config & config)> m_tracking_mode_pre_changed_ls = [](const Config &) {};
  std::function<void(const std::string & state, const bool visible)> m_status_changed_ls = [](const std::string &, const bool) {};
  std::function<void(const std::string & reason)> m_aborted_ls = [](const std::string &) {};
  std::function<void(const std::string & region, const StringVector & region_data)> m_region_ls =
      [](const std::string & region, const StringVector & region_data) {};

  std::function<void(const std::string & debug_log)> m_debug_logger_ls = [](const std::string &) {};
};

#endif // TRACKING_PARSER_H
