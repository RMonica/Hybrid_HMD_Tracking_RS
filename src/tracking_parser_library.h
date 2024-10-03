/*
 * Copyright 2022-2024 Riccardo Monica
 * 
 * This software is distributed under the 3-clause BSD license.
 * You should have received a copy of the 3-clause BSD license
 * along with this software. If not, see
 * <https://opensource.org/license/bsd-3-clause>
 */

#ifndef TRACKING_PARSER_LIBRARY_H
#define TRACKING_PARSER_LIBRARY_H

#if _MSC_VER // this is defined when compiling with Visual Studio
  #define EXPORT_API __declspec(dllexport) // Visual Studio needs annotating exported functions with this
#else
  #define EXPORT_API
#endif

#include <stdint.h>

extern "C"
{
  enum TrackingParserState
  {
    STATE_TRACKING_NONE      = -1,
    STATE_TRACKER_RESETTING  = 0,
    STATE_MOTIVE_HIDDEN      = 1,
    STATE_MOTIVE_VISIBLE     = 2,
    STATE_MOTIVE_VISIBLE_END = 3,
  };

  EXPORT_API bool TrackingParserLoadFile(const uint8_t * filename);
  EXPORT_API bool TrackingParserNextFrame(); // false if no frame
  EXPORT_API bool TrackingParserIsHidden();
  EXPORT_API bool TrackingParserIsEnded();
  EXPORT_API int TrackingParserGetState(); // returns enum state, -1 if unchanged
  EXPORT_API void TrackingParserGetMotive(float * motive_mat);
  EXPORT_API void TrackingParserGetOculus(float * oculus_mat);
  EXPORT_API int TrackingParserGetRegion(uint8_t * region, int max_len); // returns actual num bytes
  EXPORT_API void TrackingParserSetCalibrationMatrix(float * calib_mat);
}

#endif // TRACKING_PARSER_LIBRARY_H
