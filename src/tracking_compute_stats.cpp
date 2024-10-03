/*
 * Copyright 2022-2024 Riccardo Monica
 * 
 * This software is distributed under the 3-clause BSD license.
 * You should have received a copy of the 3-clause BSD license
 * along with this software. If not, see
 * <https://opensource.org/license/bsd-3-clause>
 */

#include <fstream>
#include <sstream>
#include <iostream>
#include <stdint.h>
#include <string>
#include <vector>

typedef uint64_t uint64;
typedef std::vector<std::string> StringVector;

struct ParsedLine
{
  uint64 frame_count;
  float pos_error = -1; // -1 if none
  float rot_error = -1;
  float pos_diff = -1;
  float rot_diff = -1;
  float pos_vib = -1;
  float rot_vib = -1;
  double comp_time = -1;
  bool lost = false;
};

template <typename T>
T MaybeParse(std::istream & istr, const T default_value)
{
  std::string w;
  istr >> w;
  if (istr)
  {
    std::istringstream iss(w);
    T f;
    iss >> f;
    if (iss)
      return f;
  }
  return default_value;
}

ParsedLine ParseLine(const std::string & line)
{
  std::istringstream istr(line);
  ParsedLine result;

  istr >> result.frame_count;
  result.pos_error = MaybeParse(istr, -1.0f);
  result.rot_error = MaybeParse(istr, -1.0f);
  result.pos_diff = MaybeParse(istr, -1.0f);
  result.rot_diff = MaybeParse(istr, -1.0f);
  result.pos_vib = MaybeParse(istr, -1.0f);
  result.rot_vib = MaybeParse(istr, -1.0f);
  std::string lost_str;
  istr >> lost_str;
  result.lost = (lost_str == "YES");
  result.comp_time = MaybeParse(istr, -1.0);

  if (!istr)
  {
    std::cout << "Error: Unable to parse line: \"" << line << "\"" << std::endl;
    std::exit(5);
  }

  return result;
}

StringVector ReadFileLines(std::istream & ifile)
{
  StringVector result;
  std::string line;
  while (std::getline(ifile, line))
    result.push_back(line);
  return result;
}

int main(int argc, char ** argv)
{
  if (argc < 7)
  {
    std::cout << "Usage: tracking_compute_stats errors.txt range_min range_max errors_stats.txt MODE line" << std::endl;
    std::exit(1);
  }

  const std::string errors_filename = argv[1];
  const std::string range_min_str = argv[2];
  const std::string range_max_str = argv[3];
  const std::string errors_stats_filename = argv[4];
  const std::string mode = argv[5];
  const std::string outfile_line_str = argv[6];

  StringVector error_stats_strings;
  {
    std::cout << "Reading file " << errors_stats_filename << std::endl;
    std::ifstream esfile(errors_stats_filename.c_str());
    if (!esfile)
      std::cout << "  File not existing, it will be created." << std::endl;
    if (esfile)
      error_stats_strings = ReadFileLines(esfile);
  }

  uint64 range_min;
  uint64 range_max;
  uint64 outfile_line;
  try
  {
    range_min = std::stoi(range_min_str);
    range_max = std::stoi(range_max_str);
    outfile_line = std::stoi(outfile_line_str);
  }
  catch (std::exception e)
  {
    std::cout << "Input parsing error: " << e.what() << std::endl;
    std::exit(1);
  }

  std::cout << "range: [" << range_min << " - " << range_max << "]" << std::endl;

  std::cout << "Opening file: " << errors_filename << std::endl;
  std::ifstream efile(errors_filename.c_str());
  if (!efile)
  {
    std::cout << "Could not open input file: " << errors_filename << std::endl;
    std::exit(2);
  }

  float pos_error_sum = 0.0f;
  float rot_error_sum = 0.0f;
  uint64 error_count = 0;

  float pos_vib_sum = 0.0f;
  float rot_vib_sum = 0.0f;
  uint64 vib_count = 0;

  double comp_time_sum = 0.0f;
  uint64 comp_time_count = 0;

  uint64 frame_count = 0;
  bool header = true;
  std::string line;
  while (std::getline(efile, line))
  {
    if (header)
    {
      header = false;
      continue;
    }

    if (frame_count >= range_min && frame_count < range_max)
    {
      ParsedLine parsed_line = ParseLine(line);

      if (parsed_line.rot_error != -1.0f && parsed_line.pos_error != -1.0f)
      {
        pos_error_sum += parsed_line.pos_error;
        rot_error_sum += parsed_line.rot_error;
        error_count++;
      }

      if (parsed_line.pos_vib != -1.0f && parsed_line.rot_vib != -1.0f)
      {
        pos_vib_sum += parsed_line.pos_vib;
        rot_vib_sum += parsed_line.rot_vib;
        vib_count++;
      }

      if (parsed_line.comp_time != -1.0)
      {
        comp_time_sum += parsed_line.comp_time;
        comp_time_count++;
      }
    }

    frame_count++;
  }

  pos_error_sum /= error_count;
  rot_error_sum /= error_count;

  pos_vib_sum /= vib_count;
  rot_vib_sum /= vib_count;

  comp_time_sum /= comp_time_count;

  std::cout << "Error: p " << pos_error_sum << " r " << rot_error_sum << std::endl;
  std::cout << "Vib: p " << pos_vib_sum << " r " << rot_vib_sum << std::endl;

  if (error_stats_strings.size() <= outfile_line)
    error_stats_strings.resize(outfile_line + 1);
  error_stats_strings[outfile_line] =
    mode + "\t" + std::to_string(range_min) + "\t" + std::to_string(range_max) + "\t" +
    std::to_string(pos_error_sum) + "\t" + std::to_string(rot_error_sum) + "\t" +
    std::to_string(pos_vib_sum) + "\t" + std::to_string(rot_vib_sum) + "\t" +
    std::to_string(comp_time_sum * 1000.0);

  std::cout << "Writing file: " << errors_stats_filename << std::endl;
  std::ofstream ofile(errors_stats_filename.c_str());
  if (!ofile)
  {
    std::cout << "Could not create output file: " << errors_stats_filename << std::endl;
    std::exit(3);
  }
  for (const std::string & s : error_stats_strings)
    ofile << s << "\n";

  return 0;
}
