/*
 * Copyright 2022-2024 Riccardo Monica
 * 
 * This software is distributed under the 3-clause BSD license.
 * You should have received a copy of the 3-clause BSD license
 * along with this software. If not, see
 * <https://opensource.org/license/bsd-3-clause>
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdint.h>
#include <map>
#include <vector>
#include <set>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "tracking_library.h"
#include "tracking_interface.h"

#include "tracking_parser_utils.h"
#include "tracking_parser.h"

class StatsCollector
{
  public:
  typedef uint64_t uint64;
  using ConfigResultMap = TrackingParser::ConfigResultMap;
  typedef TrackingParser::ConfigResultPair ConfigResultPair;
  typedef TrackingParser::ConfigParameterMap ConfigParameterMap;
  typedef std::vector<ConfigResultMap> ConfigResultMapVector;

  struct ThresholdStat
  {
    Tracking::Parameters parameter;
    float threshold;
    uint64 total;
    uint64 detections;

    ThresholdStat(Tracking::Parameters parameter, const float threshold)
    {
      this->threshold = threshold;
      this->parameter = parameter;
      total = 0;
      detections = 0;
    }
  };

  typedef std::map<float, ThresholdStat> ThresholdStatMap;
  typedef std::pair<float, ThresholdStat> ThresholdStatPair;

  struct ModeParameter
  {
    Tracking::Mode mode;
    Tracking::Parameters parameter;

    explicit ModeParameter(Tracking::Mode mode, Tracking::Parameters parameter)
    {
      this->parameter = parameter;
      this->mode = mode;
    }

    bool operator<(const ModeParameter & other) const
    {
      if (int(mode) < int(other.mode))
        return true;
      if (int(mode) > int(other.mode))
        return false;

      if (int(parameter) < int(other.parameter))
        return true;
      if (int(parameter) > int(other.parameter))
        return false;

      return false;
    }

    bool operator==(const ModeParameter & other) const
    {
      return this->mode == other.mode && this->parameter == other.parameter;
    }

    bool operator>(const ModeParameter & other) const
    {
      return !(*this < other) && !(*this == other);
    }
  };

  struct ParameterStat
  {
    Tracking::Mode mode;
    Tracking::Parameters parameter;
    ThresholdStatMap thresholds;

    ParameterStat(Tracking::Mode mode, Tracking::Parameters parameter)
    {
      this->parameter = parameter;
      this->mode = mode;
    }
  };

  typedef std::map<ModeParameter, ParameterStat> ParameterStatMap;
  typedef std::pair<ModeParameter, ParameterStat> ParameterStatPair;
  typedef std::set<ModeParameter> ModeParameterSet;

  explicit StatsCollector(const uint64 num_files)
  {
    m_num_files = num_files;
  }

  void AddParsingResult(const ConfigResultMap & parsing_result, const ModeParameterSet & mode_parameters_only)
  {
    m_parsing_results.push_back(parsing_result);

    for (ConfigResultMap::const_iterator iter = parsing_result.begin(); iter != parsing_result.end(); iter++)
    {
      const ConfigResultPair & conf = *iter;

      const Tracking::Mode evaluated_mode = conf.first.mode;
      if (evaluated_mode == Tracking::MODE_MOTIVE_FIRST)
        continue;
      if (evaluated_mode == Tracking::MODE_OCULUS_FIRST)
        continue;

      const Tracking::Parameters evaluated_parameter = conf.first.evaluated_parameter;
      const ModeParameter mode_parameter(evaluated_mode, evaluated_parameter);
      if (!mode_parameters_only.empty() && mode_parameters_only.find(mode_parameter) == mode_parameters_only.end())
        continue; // skip this mode-parameter

      if (m_stats.find(mode_parameter) == m_stats.end())
      {
        m_stats.insert(ParameterStatPair(mode_parameter, ParameterStat(evaluated_mode, evaluated_parameter)));
      }

      ThresholdStatMap & thresholds = m_stats.find(mode_parameter)->second.thresholds;

      const ConfigParameterMap & parameters = conf.first.parameters;
      float value = 0.0f;
      if (evaluated_parameter != Tracking::PARAM_NONE &&
          parameters.find(evaluated_parameter) != parameters.end())
      {
        value = parameters.find(evaluated_parameter)->second.value;
      }

      if (thresholds.find(value) == thresholds.end())
      {
        thresholds.insert(ThresholdStatPair(value, ThresholdStat(evaluated_parameter, value)));
      }

      ThresholdStat & stat = thresholds.find(value)->second;

      if (conf.second.detected)
        stat.detections++;
      if (conf.second.total)
        stat.total++;
    }

    // motive first is a special case
    if (mode_parameters_only.empty() ||
        mode_parameters_only.find(ModeParameter(Tracking::MODE_MOTIVE_FIRST, Tracking::PARAM_NONE)) != mode_parameters_only.end())
    {
      for (ConfigResultMap::const_iterator iter = parsing_result.begin(); iter != parsing_result.end(); iter++)
      {
        const ConfigResultPair & conf = *iter;

        const Tracking::Mode evaluated_mode = conf.first.mode;

        if (evaluated_mode == Tracking::MODE_MOTIVE_FIRST)
        {
          m_motive_first_detected += conf.second.detected;
          m_motive_first_total += conf.second.total;
        }
      }
    }

    // oculus first is a special case
    if (mode_parameters_only.empty() ||
        mode_parameters_only.find(ModeParameter(Tracking::MODE_OCULUS_FIRST, Tracking::PARAM_NONE)) != mode_parameters_only.end())
    {
      for (ConfigResultMap::const_iterator iter = parsing_result.begin(); iter != parsing_result.end(); iter++)
      {
        const ConfigResultPair & conf = *iter;

        const Tracking::Mode evaluated_mode = conf.first.mode;

        if (evaluated_mode == Tracking::MODE_OCULUS_FIRST)
        {
          m_oculus_first_detected += conf.second.detected;
          m_oculus_first_total += conf.second.total;
        }
      }
    }
  }

  void PrintStats()
  {
    for (const ParameterStatPair & p : m_stats)
    {
      std::cout << "Mode: " << ModeToString(p.second.mode) << " ";
      std::cout << "Parameter: " << ParameterToString(p.second.parameter) << std::endl;
      const ThresholdStatMap & thresholds = p.second.thresholds;
      for (const ThresholdStatPair & t : thresholds)
      {
        std::cout << t.second.threshold << "\t: " << t.second.detections << " / " << t.second.total << std::endl;
      }
    }

    std::cout << "Motive first: " << m_motive_first_detected << "/" << m_motive_first_total << std::endl;
    std::cout << "Oculus first: " << m_oculus_first_detected << "/" << m_oculus_first_total << std::endl;
  }

  void SaveStats(const std::string & filename_prefix)
  {
    for (const ParameterStatPair & p : m_stats)
    {
      const std::string mode = ModeToString(p.second.mode);
      const std::string parameter = ParameterToString(p.second.parameter);
      const std::string filename = filename_prefix + "_" + mode + "_" + parameter + ".txt";
      std::ofstream ofile(filename.c_str());

      const ThresholdStatMap & thresholds = p.second.thresholds;
      for (const ThresholdStatPair & t : thresholds)
      {
        ofile << t.second.threshold << "\t" << t.second.detections << "\t" << t.second.total << std::endl;
      }
    }

    if (m_motive_first_total != 0)
    {
      const std::string filename = filename_prefix + "_" + "MOTIVE_FIRST" + ".txt";
      std::ofstream ofile(filename.c_str());
      ofile << m_motive_first_detected << "\t" << m_motive_first_total << std::endl;
    }
    if (m_oculus_first_total != 0)
    {
      const std::string filename = filename_prefix + "_" + "OCULUS_FIRST" + ".txt";
      std::ofstream ofile(filename.c_str());
      ofile << m_oculus_first_detected << "\t" << m_oculus_first_total << std::endl;
    }
  }

  private:
  uint64 m_num_files;
  ConfigResultMapVector m_parsing_results;

  ParameterStatMap m_stats;

  uint64 m_motive_first_detected = 0;
  uint64 m_motive_first_total = 0;

  uint64 m_oculus_first_detected = 0;
  uint64 m_oculus_first_total = 0;
};


int main(int argc, char ** argv)
{
  if (argc < 2)
  {
    std::cout << "Usage: tracking_parser {filename1.txt ...} [-o output_file_prefix] "
                 "{-s selected_method,selected_param}" << std::endl;
    std::exit(1);
  }

  std::string output_file_prefix;
  std::vector<std::string> filenames;
  std::set<StatsCollector::ModeParameter> mode_parameters_only;
  for (int i = 1; i < argc; i++)
  {
    const std::string filename = argv[i];
    if (filename == "-o")
    {
      if (i + 1 < argc)
      {
        output_file_prefix = argv[i + 1];
        i++;
      }
      else
      {
        std::cout << "Expected filename after -o in command line." << std::endl;
        std::exit(1);
      }
    }
    else if (filename == "-s")
    {
      if (i + 1 < argc)
      {
        std::string mps = argv[i + 1];
        i++;
        std::size_t comma_pos = mps.find(',');
        if (comma_pos == std::string::npos)
        {
          std::cout << "Expected comma after -s in command line." << std::endl;
          std::exit(1);
        }

        std::string mode_str = mps.substr(0, mps.find(','));
        std::string parameter_str = mps.substr(mps.find(',') + 1);

        try
        {
          StatsCollector::ModeParameter par(StringToMode(mode_str), StringToParameter(parameter_str));
          mode_parameters_only.insert(par);
        }
        catch (const std::string & e)
        {
          std::cout << e << std::endl;
          std::exit(5);
        }
      }
      else
      {
        std::cout << "Expected mode,parameter after -s in command line." << std::endl;
        std::exit(1);
      }
    }
    else
      filenames.push_back(filename);
  }

  StatsCollector stats_collector(filenames.size());
  for (const std::string & filename : filenames)
  {
    std::cout << "Opening file: " << filename << std::endl;
    std::ifstream ifile(filename.c_str());
    if (!ifile)
    {
      std::cout << "Could not open file: " << filename << std::endl;
      std::exit(2);
    }

    TrackingParser parser;
    try
    {
      parser.Parse(ifile);
    }
    catch (std::string str)
    {
      std::cout << "Parsing error in file : " << str << std::endl;
      std::exit(3);
    }

    const TrackingParser::ConfigResultMap result = parser.GetConfigResultMap();
    stats_collector.AddParsingResult(result, mode_parameters_only);
  }

  stats_collector.PrintStats();

  if (!output_file_prefix.empty())
    stats_collector.SaveStats(output_file_prefix);

  return 0;
}
