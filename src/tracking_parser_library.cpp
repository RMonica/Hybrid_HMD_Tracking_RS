/*
 * Copyright 2022-2024 Riccardo Monica
 * 
 * This software is distributed under the 3-clause BSD license.
 * You should have received a copy of the 3-clause BSD license
 * along with this software. If not, see
 * <https://opensource.org/license/bsd-3-clause>
 */

#include "tracking_parser_library.h"

#include "tracking_parser.h"
#include "tracking_commons.h"

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <stdint.h>
#include <string>
#include <cstring>
#include <vector>

typedef std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f> > Affine3fVector;
typedef std::vector<bool> BoolVector;
typedef std::vector<TrackingParserState> StateVector;
typedef std::vector<std::string> StringVector;
typedef std::vector<int64_t> Int64Vector;

typedef uint64_t uint64;
typedef int64_t int64;

struct TrackingParserStatus
{
  Affine3fVector motive_matrices;
  Affine3fVector oculus_matrices;

  BoolVector hidden;
  StateVector states;

  TrackingParserState state = STATE_TRACKING_NONE;
  bool ended = false;

  std::string region;
  StringVector regions;
  Int64Vector frame_to_region_idx;

  uint64 current_frame = 0;
};

static TrackingParserStatus status;

static Eigen::Affine3f calibration_matrix = Eigen::Affine3f::Identity();

void OnFrame(const uint64 frame)
{
  status.states.push_back(status.state);
  status.state = STATE_TRACKING_NONE;
  if (status.region != "")
  {
    status.regions.push_back(status.region);
    status.region = "";
  }
  status.frame_to_region_idx.push_back(int64_t(status.regions.size()) - 1);
}

void OnMotiveMatrix(const Eigen::Affine3f & m)
{
  status.motive_matrices.push_back(m);
}

void OnOculusMatrix(const Eigen::Affine3f & m)
{
  status.oculus_matrices.push_back(m);
}

void OnReposition()
{
}

void OnAborted(const std::string & reason)
{
}

void OnModeChanged(const TrackingParser::Config & config)
{
}

void OnModePreChanged(const TrackingParser::Config & config)
{
}

void OnStatusChanged(const std::string & status_str, const bool visible)
{
  if (status_str == "TRACKER_RESETTING")
  {
    status.state = STATE_TRACKER_RESETTING;
  }

  if (status_str == "MOTIVE_HIDDEN")
  {
    status.state = STATE_MOTIVE_HIDDEN;
  }

  if (status_str == "MOTIVE_VISIBLE_END")
  {
    status.state = STATE_MOTIVE_VISIBLE_END;
  }

  if (status_str == "MOTIVE_VISIBLE")
  {
    status.state = STATE_MOTIVE_VISIBLE;
  }

  status.hidden.push_back(!visible);
}

void OnRegion(const std::string & region, const StringVector & region_data)
{
  status.region = region;
}

EXPORT_API bool TrackingParserLoadFile(const uint8_t * filename)
{
  std::ifstream ifile((const char *)filename);
  if (!ifile)
    return false;

  status = TrackingParserStatus();

  TrackingParser parser;
  parser.SetFrameListener(OnFrame);
  parser.SetAbortedListener(OnAborted);
  parser.SetModeChangedListener(OnModeChanged);
  parser.SetMotiveListener(OnMotiveMatrix);
  parser.SetOculusListener(OnOculusMatrix);
  parser.SetRegionListener(OnRegion);
  parser.SetRepositionListener(OnReposition);
  parser.SetStatusChangedListener(OnStatusChanged);
  parser.SetModePreChangedListener(OnModePreChanged);
  parser.SetDebugLogListener([](const std::string &) {}); // disable parser debug log
  parser.SetCalibrationMatrix(calibration_matrix);

  try
  {
    parser.Parse(ifile);
  }
  catch (std::string str)
  {
    return false;
  }

  return true;
}

EXPORT_API bool TrackingParserNextFrame()
{
  if (status.current_frame + 1 >= status.motive_matrices.size())
  {
    status.ended = true;
    return false;
  }

  status.current_frame++;
  return true;
}

EXPORT_API bool TrackingParserIsHidden()
{
  return status.hidden[status.current_frame];
}

EXPORT_API void TrackingParserGetMotive(float * motive_mat_out)
{
  Eigen::Affine3f motive_mat = status.motive_matrices[status.current_frame];
  Tracking::Affine3fToFloatv(motive_mat, motive_mat_out);
}

EXPORT_API void TrackingParserGetOculus(float * oculus_mat_out)
{
  Eigen::Affine3f oculus_mat = status.oculus_matrices[status.current_frame];
  Tracking::Affine3fToFloatv(oculus_mat, oculus_mat_out);
}

EXPORT_API void TrackingParserSetCalibrationMatrix(float * calib_mat)
{
  calibration_matrix = Tracking::FloatvToAffine3f(calib_mat);
}

EXPORT_API int TrackingParserGetState()
{
  return status.states[status.current_frame];
}

EXPORT_API int TrackingParserGetRegion(uint8_t * region, int max_len)
{
  const int64_t region_idx = status.frame_to_region_idx[status.current_frame];
  // we must return NEXT region, thus +1
  if (region_idx + 1 >= status.regions.size())
    return 0;
  if (region_idx + 1 < 0)
    return 0;
  const std::string & r = status.regions[region_idx + 1];
  std::strncpy((char *)region, r.c_str(), max_len);
  return r.size();
}

EXPORT_API bool TrackingParserIsEnded()
{
  return status.ended;
}



