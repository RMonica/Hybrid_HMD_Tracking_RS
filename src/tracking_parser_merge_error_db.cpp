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

typedef std::map<std::string, std::string> StringStringMap;
typedef std::pair<std::string, std::string> StringStringPair;

int main(int argc, char ** argv)
{
  if (argc < 2)
  {
    std::cout << "Usage: tracking_parser_merge_error_db {file1.txt file2.txt...} [-o output_file.txt]" << std::endl;
    std::exit(1);
  }

  StringVector input_filenames;
  std::string output_filename;

  for (uint64 p = 1; p < argc; p++)
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
    else
    {
      input_filenames.push_back(argv[p]);
    }
  }

  StringStringMap input_data;
  for (const std::string input_filename : input_filenames)
  {
    std::ifstream ifile(input_filename);
    std::string line;
    while (std::getline(ifile, line))
    {
      if (line == "")
        continue;
      if (line[0] == '#')
        continue;

      std::istringstream istr(line);
      std::string key;
      istr >> key;
      std::string data = line.substr(key.size());
      input_data[key] += data;
    }
  }

  if (!output_filename.empty())
  {
    std::ofstream ofile(output_filename);
    for (StringStringPair p : input_data)
    {
      ofile << p.first << p.second << std::endl;
    }
  }

  for (StringStringPair p : input_data)
  {
    std::cout << p.first << p.second << std::endl;
  }

  return 0;
}
