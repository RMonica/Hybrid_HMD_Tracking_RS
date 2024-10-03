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
#include <memory>
#include <map>
#include <iomanip>

#include "tracking_parser_utils.h"
#include "tracking_parser.h"

typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > Vector3fVector;
typedef std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f> > Affine3fVector;
typedef std::vector<std::string> StringVector;
typedef std::vector<float> FloatVector;

typedef uint64_t uint64;
typedef uint32_t uint32;
typedef int64_t int64;
typedef int32_t int32;
#define _1 (std::placeholders::_1)
#define _2 (std::placeholders::_2)
typedef std::vector<uint64> Uint64Vector;

typedef std::map<std::string, float> StringFloatMap;
typedef std::pair<std::string, float> StringFloatPair;

struct AlgoUser
{
  std::string algorithm;
  std::string user;

  AlgoUser(const std::string a = "", const std::string u = "")
  {
    algorithm = a;
    user = u;
  }

  bool operator<(const AlgoUser & other) const
    {return std::pair(algorithm, user) < std::pair(other.algorithm, other.user); }
  bool operator==(const AlgoUser & other) const
    {return std::pair(algorithm, user) == std::pair(other.algorithm, other.user); }
  bool operator!=(const AlgoUser & other) const
    {return std::pair(algorithm, user) != std::pair(other.algorithm, other.user); }
};

typedef std::map<AlgoUser, float> AlgoUserFloatMap;
typedef std::pair<AlgoUser, float> AlgoUserFloatPair;

const float PI = std::acos(-1.0f);

class StatsComputation
{
  public:
  StatsComputation()
  {
    m_counter = 0;
  }

  void AddStats(const StringFloatMap & eval_stats, const StringFloatMap signal_stats)
  {
    for (StringFloatPair ep : eval_stats)
    {
      if (m_evaluation_result.find(ep.first) == m_evaluation_result.end())
        m_evaluation_result[ep.first] = 0.0f;
      m_evaluation_result[ep.first] += ep.second;
    }

    for (StringFloatPair ss : signal_stats)
    {
      if (m_signal_result.find(ss.first) == m_signal_result.end())
        m_signal_result[ss.first] = 0.0f;
      m_signal_result[ss.first] += ss.second;
    }

    m_counter++;
  }

  void AddDetailedStats(const AlgoUserFloatMap & eval_stats, const AlgoUserFloatMap & signal_stats)
  {
    m_evaluation_details.insert(eval_stats.begin(), eval_stats.end());
    m_signal_details.insert(signal_stats.begin(), signal_stats.end());
  }

  void PrintDetailedSignalStats(std::ostream & ofile = std::cout) const
  {
    for (AlgoUserFloatMap::const_iterator iter = m_signal_details.begin(); iter != m_signal_details.end(); iter++)
    {
      ofile << iter->first.algorithm << "\t" << iter->first.user << "\t" << iter->second << std::endl;
    }
  }

  void PrintStats(std::ostream & ofile = std::cout)
  {
    for (StringFloatPair er : m_evaluation_result)
    {
      if (m_signal_result.find(er.first) == m_signal_result.end())
      {
        ofile << "Error: could not find key " << er.first << "in signal results." << std::endl;
        continue;
      }

      const float sv = m_signal_result[er.first] / (m_counter * 2.0f);
      const float ev = m_evaluation_result[er.first] / m_counter;

      ofile << er.first << "\t" << sv << "\t" << ev << std::endl;
    }
  }

  private:

  StringFloatMap m_evaluation_result;
  StringFloatMap m_signal_result;

  AlgoUserFloatMap m_evaluation_details;
  AlgoUserFloatMap m_signal_details;

  uint64 m_counter;
};

class ParserListener
{
  public:
  ParserListener()
  {
    m_padding_string = "mode_directed_complementarykw";
    m_counter = 0;
  }

  void SetUserName(const std::string & n)
  {
    m_username = n;
  }

  void SetEvaluationLine(const FloatVector & el)
  {
    m_evaluation_line = el;
  }

  void SetSignalLine(const FloatVector & sl)
  {
    m_signal_line = sl;
  }

  void OnFrame(const uint64 frame)
  {
    //std::cout << "Frame: " << frame << std::endl;
  }

  void OnReposition()
  {
    if(!m_is_repositioned)
      std::cout << "Reposition" << std::endl;
    m_is_repositioned = true;
  }

  void OnModePreChanged(const TrackingParser::Config & config)
  {
    std::ostringstream ostr;

    const std::string mode_name = ModeToString(config.mode);
    std::string omodename;
    for (char c : mode_name)
      omodename += std::tolower(c);
    ostr << omodename;

    if (config.kalman_wrapped)
      ostr << "kw";

    for (const TrackingParser::ConfigParameterPair & pp : config.parameters)
    {
      const TrackingParser::ConfigParameter & p = pp.second;
      const std::string parameter_name = ParameterToString(p.parameter);
      std::string opn;
      for (char c : parameter_name)
        opn += std::tolower(c);
      ostr << opn;

      ostr << p.value;
    }

    const std::string pre_config = ostr.str();
    const AlgoUser algo_user(pre_config, m_username);

    if (m_counter < m_signal_line.size())
    {
      ostr << "\t";
      ostr << m_signal_line[m_counter];
    }

    if (m_counter < m_evaluation_line.size())
    {
      ostr << "\t";
      ostr << m_evaluation_line[m_counter];
    }

    if (m_prev_pre_config != pre_config && pre_config != m_padding_string)
    {
      std::cout << ostr.str() << std::endl;

      m_evaluation_result[pre_config] = m_evaluation_line[m_counter];
      m_signal_result[pre_config] = m_signal_line[m_counter];

      m_evaluation_details[algo_user] = m_evaluation_line[m_counter];
      m_signal_details[algo_user] = m_signal_line[m_counter];

      m_counter++;
    }

    m_prev_pre_config = pre_config;

    m_is_repositioned = false;
  }

  AlgoUserFloatMap GetDetailedEvaluationStats() const {return m_evaluation_details; }
  AlgoUserFloatMap GetDetailedSignalStats() const {return m_signal_details; }

  StringFloatMap GetEvaluationStats() const {return m_evaluation_result; }
  StringFloatMap GetSignalStats() const {return m_signal_result; }

  private:

  std::string m_prev_pre_config;
  std::string m_padding_string;

  uint64 m_counter;

  std::string m_username;

  bool m_is_repositioned;

  FloatVector m_evaluation_line;
  FloatVector m_signal_line;

  StringFloatMap m_evaluation_result;
  StringFloatMap m_signal_result;

  AlgoUserFloatMap m_evaluation_details;
  AlgoUserFloatMap m_signal_details;
};

class FileLines
{
  public:
  FileLines(std::istream & ifile)
  {
    std::string line;
    while (std::getline(ifile, line))
      m_file_lines.push_back(line);
  }

  std::string GetLine(const uint64 i) const
  {
    return m_file_lines[i];
  }

  Uint64Vector GetLineAsUints(const uint64 i)
  {
    Uint64Vector result;
    const std::string & line = m_file_lines[i];
    std::istringstream istr(line);
    uint64 v;
    while (istr >> v)
      result.push_back(v);
    return result;
  }

  FloatVector GetLineAsFloats(const uint64 i)
  {
    FloatVector result;
    const std::string & line = m_file_lines[i];
    std::istringstream istr(line);
    uint64 v;
    while (istr >> v)
      result.push_back(v);
    return result;
  }

  FloatVector GetLineAsRankedFloats(const uint64 idx)
  {
    const FloatVector float_line = GetLineAsFloats(idx);
    const uint64 size = float_line.size();
    Uint64Vector ranks(size);
    for (uint64 i = 0; i < size; i++)
      ranks[i] = i;
    std::sort(ranks.begin(), ranks.end(),
              [float_line](const uint64 a, const uint64 b) -> bool
              {
                return float_line[a] < float_line[b];
              });
    FloatVector reversed_ranks(size);
    for (uint64 i = 0; i < size; i++)
    {
      reversed_ranks[ranks[i]] = i;
    }

    return reversed_ranks;
  }

  FloatVector GetLineAsRankedAveragedFloats(const uint64 idx)
  {
    const Uint64Vector u_line = GetLineAsUints(idx);
    const FloatVector rank_line = GetLineAsRankedFloats(idx);
    const uint64 size = u_line.size();

    FloatVector result(size, 0.0f);

    // average ranks of same number
    for (uint64 i = 0; i < size; i++)
    {
      const uint64 v = u_line[i];
      float avg = 0.0f;
      uint64 counter = 0;
      for (uint64 h = 0; h < size; h++)
        if (u_line[h] == v)
        {
          counter++;
          avg += rank_line[h];
        }
      avg /= float(counter);
      result[i] = avg;
    }

    return result;
  }

  private:
  StringVector m_file_lines;
};

int main(int argc, char ** argv)
{
  if (argc < 4)
  {
    std::cout << "Usage: tracking_parser_get_sequence list_file.txt signal_file.txt eval_file.txt"
                 "[-o output_file.txt] [-detail-output output_detail_file.txt]" << std::endl;
    std::exit(1);
  }

  const std::string list_filename = argv[1];
  const std::string signal_filename = argv[2];
  const std::string eval_filename = argv[3];
  std::string output_filename;
  std::string detail_output_filename;

  for (uint64 p = 4; p < argc; p++)
  {
    std::string cmd = argv[p];
    if (cmd == "-o")
    {
      p++;
      if (p >= argc)
      {
        std::cout << "Expected filename after -o" << std::endl;
        std::exit(5);
      }
      output_filename = argv[p];
    }
    else if (cmd == "-detail-output")
    {
      p++;
      if (p >= argc)
      {
        std::cout << "Expected filename after -detail-output" << std::endl;
        std::exit(5);
      }
      detail_output_filename = argv[p];
    }
    else
    {
      std::cout << "Unknown option: " << cmd << std::endl;
    }
  }

  std::cout << "Opening list file: " << list_filename << std::endl;
  std::ifstream list_file(list_filename);
  if (!list_file)
  {
    std::cout << "Could not open file: " << list_filename << std::endl;
    std::exit(1);
  }

  std::cout << "Opening file: " << signal_filename << std::endl;
  std::ifstream sfile(signal_filename.c_str());
  if (!sfile)
  {
    std::cout << "Could not open file: " << signal_filename << std::endl;
    std::exit(2);
  }
  FileLines sfilelines(sfile);

  std::cout << "Opening file: " << eval_filename << std::endl;
  std::ifstream efile(eval_filename.c_str());
  if (!efile)
  {
    std::cout << "Could not open file: " << eval_filename << std::endl;
    std::exit(2);
  }
  FileLines efilelines(efile);

  StatsComputation stats_computation;

  std::string list_line;
  uint64 user_counter = 1;
  while (std::getline(list_file, list_line))
  {
    if (list_line.empty())
      continue;
    if (list_line[0] == '#')
      continue;
    std::cout << "List line is: " << list_line << std::endl;
    std::istringstream list_line_stream(list_line);

    std::string log_filename;
    list_line_stream >> log_filename;

    std::string fake_user_name;
    {
      std::ostringstream ss;
      ss << std::setw(2) << std::setfill('0') << user_counter;
      fake_user_name = "user" +  ss.str();
    }
    user_counter++;

    uint64 signal_line;
    list_line_stream >> signal_line;
    std::cout << "Signal line is: " << signal_line << std::endl;
    uint64 eval_line;
    list_line_stream >> eval_line;
    std::cout << "Eval line is: " << eval_line << std::endl;

    if (!list_line_stream)
    {
      std::cout << "Error while parsing list line: " << list_line << std::endl;
      std::exit(4);
    }

    std::cout << "Opening file: " << log_filename << std::endl;
    std::ifstream ifile(log_filename.c_str());
    if (!ifile)
    {
      std::cout << "Could not open file: " << log_filename << std::endl;
      std::exit(2);
    }

    ParserListener pl;
    {
      FloatVector l = efilelines.GetLineAsFloats(eval_line);
      FloatVector rl = efilelines.GetLineAsRankedFloats(eval_line);
      FloatVector ral = efilelines.GetLineAsRankedAveragedFloats(eval_line);
      std::cout << "float line: ";
      for (float v : l)
        std::cout << v << "\t";
      std::cout << std::endl;
      std::cout << "ranks line: ";
      for (float v : rl)
        std::cout << v << "\t";
      std::cout << std::endl;
      std::cout << "ral   line: ";
      for (float v : ral)
        std::cout << v << "\t";
      std::cout << std::endl;
    }
    pl.SetEvaluationLine(efilelines.GetLineAsRankedAveragedFloats(eval_line));
    pl.SetSignalLine(sfilelines.GetLineAsFloats(signal_line));
    pl.SetUserName(fake_user_name);

    TrackingParser parser;
    parser.SetFrameListener(std::bind(&ParserListener::OnFrame, &pl, _1));
    parser.SetModePreChangedListener(std::bind(&ParserListener::OnModePreChanged, &pl, _1));
    parser.SetRepositionListener(std::bind(&ParserListener::OnReposition, &pl));
    parser.SetDebugLogListener([](const std::string &) {}); // disable parser debug log
    parser.Parse(ifile);

    stats_computation.AddStats(pl.GetEvaluationStats(), pl.GetSignalStats());
    stats_computation.AddDetailedStats(pl.GetDetailedEvaluationStats(), pl.GetDetailedSignalStats());
  }

  std::cout << "--- FINAL STATS ---" << std::endl;
  stats_computation.PrintStats();
  stats_computation.PrintDetailedSignalStats();

  if (!output_filename.empty())
  {
    std::cout << "Saving file " << output_filename << std::endl;
    std::ofstream ofile(output_filename);
    stats_computation.PrintStats(ofile);
    if (!ofile)
    {
      std::cout << "Could not save file " << output_filename << std::endl;
      std::exit(6);
    }
  }

  if (!detail_output_filename.empty())
  {
    std::cout << "Saving file " << detail_output_filename << std::endl;
    std::ofstream ofile(detail_output_filename);
    stats_computation.PrintDetailedSignalStats(ofile);
    if (!ofile)
    {
      std::cout << "Could not save file " << detail_output_filename << std::endl;
      std::exit(6);
    }
  }

  return 0;
}
