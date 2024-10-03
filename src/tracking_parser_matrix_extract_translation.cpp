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
#include <stdint.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>

typedef uint64_t uint64;

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

int main(int argc, char ** argv)
{
  if (argc < 3)
  {
    std::cout << "usage: matrix_extract_translation filename.matrix translations.txt";
    std::exit(1);
  }

  std::string ifilename(argv[1]);
  std::string ofilename(argv[2]);

  std::cout << "Opening file " << ifilename << std::endl;
  std::ifstream ifile(ifilename);
  if (!ifile)
  {
    std::cout << "Could not open file " << ifilename << std::endl;
    std::exit(2);
  }

  std::cout << "Writing file " << ofilename << std::endl;
  std::ofstream ofile(ofilename);
  if (!ifile)
  {
    std::cout << "Could not write file " << ofilename << std::endl;
    std::exit(3);
  }

  Eigen::Affine3f prev_mat = Eigen::Affine3f::Identity();
  prev_mat.linear() = Eigen::Matrix3f::Zero();
  uint64 frame_i = 0;
  while (ifile)
  {
    Eigen::Affine3f mat;
    try
    {
      mat = ParseMatrix(ifile);
    }
    catch (std::string ex)
    {
      std::cout << "ParseMatrix: " << ex << std::endl;
      break;
    }

    const bool lost = (mat.matrix() == prev_mat.matrix());

    ofile << frame_i << "\t" << mat.translation().transpose() << "\t" << (lost ? "LOST" : "OK") << "\n";

    frame_i++;
    prev_mat = mat;
  }

  return 0;
}
