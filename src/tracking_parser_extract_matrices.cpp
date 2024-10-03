/*
 * Copyright 2022-2024 Riccardo Monica
 * 
 * This software is distributed under the 3-clause BSD license.
 * You should have received a copy of the 3-clause BSD license
 * along with this software. If not, see
 * <https://opensource.org/license/bsd-3-clause>
 */

#include "tracking_parser.h"

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <stdint.h>

typedef std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f> > Affine3fVector;
typedef uint64_t uint64;

int main(int argc, char ** argv)
{
  if (argc < 4)
  {
    std::cout << "Usage: tracking_parser_extract_matrices filename.txt m_motive.txt m_oculus.txt [ALL]" << std::endl;
    std::exit(1);
  }

  std::string ifilename = argv[1];
  std::string mfilename = argv[2];
  std::string ofilename = argv[3];
  bool extract_lost_matrices = false;
  if (argc >= 5)
  {
    if (std::string(argv[4]) == "ALL")
    {
      extract_lost_matrices = true;
      std::cout << "MODE: ALL" << std::endl;
    }
  }

  std::cout << "Opening file: " << ifilename << std::endl;
  std::ifstream ifile(ifilename.c_str());
  if (!ifile)
  {
    std::cout << "Could not open file: " << ifilename << std::endl;
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

  std::cout << "Writing file: " << mfilename << std::endl;
  std::cout << "Writing file: " << ofilename << std::endl;
  std::ofstream mfile(mfilename.c_str());
  std::ofstream ofile(ofilename.c_str());

  if (!mfile)
  {
    std::cout << "Could not create file: " << mfilename << std::endl;
    std::exit(4);
  }
  if (!ofile)
  {
    std::cout << "Could not create file: " << ofilename << std::endl;
    std::exit(5);
  }

  const Affine3fVector motive_mats = parser.GetMotiveMats();
  const Affine3fVector oculus_mats = parser.GetOculusMats();

  Eigen::Affine3f prev_motive_mat = Eigen::Affine3f::Identity();

  uint64 size = std::min(motive_mats.size(), oculus_mats.size());
  for (uint64 i = 0; i < size; i++)
  {
    //std::cout << "Frame " << i << std::endl;
    const Eigen::Affine3f motive_mat = motive_mats[i];
    const Eigen::Affine3f oculus_mat = oculus_mats[i];

    if (motive_mat.matrix() == Eigen::Affine3f::Identity().matrix() ||
        oculus_mat.matrix() == Eigen::Affine3f::Identity().matrix())
      continue;
    if (!extract_lost_matrices && (motive_mat.matrix() == prev_motive_mat.matrix()))
      continue;
    prev_motive_mat = motive_mat;

    mfile << motive_mat.matrix() << "\n\n";
    ofile << oculus_mat.matrix() << "\n\n";
  }

  return 0;
}
