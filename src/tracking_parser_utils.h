/*
 * Copyright 2022-2024 Riccardo Monica
 * 
 * This software is distributed under the 3-clause BSD license.
 * You should have received a copy of the 3-clause BSD license
 * along with this software. If not, see
 * <https://opensource.org/license/bsd-3-clause>
 */

#ifndef TRACKING_PARSER_UTILS_H
#define TRACKING_PARSER_UTILS_H

#include <string>

#include "tracking_library.h"
#include "tracking_interface.h"

static Tracking::Mode StringToMode(const std::string & str)
{
  if (str == "MODE_OCULUS_ORIGIN")
      return Tracking::MODE_OCULUS_ORIGIN;
  if (str == "MODE_CONSTR_COMPLEMENTARY")
      return Tracking::MODE_CONSTR_COMPLEMENTARY;
  if (str == "MODE_CONSTR_COMPLEMENTARY_TO")
      return Tracking::MODE_CONSTR_COMPLEMENTARY_TO;
  if (str == "MODE_CONSTR_COMPLEMENTARY_RO")
      return Tracking::MODE_CONSTR_COMPLEMENTARY_RO;
  if (str == "MODE_COMPLEMENTARY")
      return Tracking::MODE_COMPLEMENTARY;
  if (str == "MODE_MOTIVE_FIRST")
      return Tracking::MODE_MOTIVE_FIRST;
  if (str == "MODE_DOUBLE_EXP")
      return Tracking::MODE_DOUBLE_EXP;
  if (str == "MODE_EXP")
      return Tracking::MODE_EXP;
  if (str == "MODE_PARTICLE_FILTER")
      return Tracking::MODE_PARTICLE_FILTER;
  if (str == "MODE_KALMAN_FILTER")
      return Tracking::MODE_KALMAN_FILTER;
  if (str == "MODE_CONSTR_KALMAN_FILTER")
      return Tracking::MODE_CONSTR_KALMAN_FILTER;
  if (str == "MODE_DIRECTED_COMPLEMENTARY")
      return Tracking::MODE_DIR_COMPLEMENTARY;
  if (str == "MODE_OCULUS_FIRST")
      return Tracking::MODE_OCULUS_FIRST;
  throw std::string("Unknown mode: " + str);
}

static std::string ModeToString(const Tracking::Mode mode)
{
  if (mode == Tracking::MODE_OCULUS_ORIGIN)
      return "MODE_OCULUS_ORIGIN";
  if (mode == Tracking::MODE_CONSTR_COMPLEMENTARY)
      return "MODE_CONSTR_COMPLEMENTARY";
  if (mode == Tracking::MODE_CONSTR_COMPLEMENTARY_RO)
      return "MODE_CONSTR_COMPLEMENTARY_RO";
  if (mode == Tracking::MODE_CONSTR_COMPLEMENTARY_TO)
      return "MODE_CONSTR_COMPLEMENTARY_TO";
  if (mode == Tracking::MODE_COMPLEMENTARY)
      return "MODE_COMPLEMENTARY";
  if (mode == Tracking::MODE_MOTIVE_FIRST)
      return "MODE_MOTIVE_FIRST";
  if (mode == Tracking::MODE_DOUBLE_EXP)
      return "MODE_DOUBLE_EXP";
  if (mode == Tracking::MODE_EXP)
      return "MODE_EXP";
  if (mode == Tracking::MODE_PARTICLE_FILTER)
      return "MODE_PARTICLE_FILTER";
  if (mode == Tracking::MODE_KALMAN_FILTER)
      return "MODE_KALMAN_FILTER";
  if (mode == Tracking::MODE_CONSTR_KALMAN_FILTER)
      return "MODE_CONSTR_KALMAN_FILTER";
  if (mode == Tracking::MODE_DIR_COMPLEMENTARY)
      return "MODE_DIRECTED_COMPLEMENTARY";
  if (mode == Tracking::MODE_OCULUS_FIRST)
      return "MODE_OCULUS_FIRST";

  return "UNKNOWN MODE";
}

static Tracking::Parameters StringToParameter(const std::string & str)
{
  if (str == "PARAM_NONE")
      return Tracking::PARAM_NONE;

  if (str == "PARAM_TRANSLATION_LIMIT")
      return Tracking::PARAM_TRANSLATION_LIMIT;
  if (str == "PARAM_TRANSLATION_DYN_LIMIT")
      return Tracking::PARAM_TRANSLATION_DYN_LIMIT;
  if (str == "PARAM_ROTATION_LIMIT")
      return Tracking::PARAM_ROTATION_LIMIT;
  if (str == "PARAM_ROTATION_DYN_LIMIT")
      return Tracking::PARAM_ROTATION_DYN_LIMIT;
  if (str == "PARAM_ROTATION_CURV_LIMIT")
      return Tracking::PARAM_ROTATION_CURV_LIMIT;
  if (str == "PARAM_ROTATION_DYN2_LIMIT")
      return Tracking::PARAM_ROTATION_DYN2_LIMIT;
  if (str == "PARAM_ROTATION_CURV2_LIMIT")
      return Tracking::PARAM_ROTATION_CURV2_LIMIT;
  if (str == "PARAM_TRANSLATION_DYN2_LIMIT")
      return Tracking::PARAM_TRANSLATION_DYN2_LIMIT;
  if (str == "PARAM_ALPHA")
      return Tracking::PARAM_ALPHA;
  if (str == "PARAM_MULT")
      return Tracking::PARAM_MULT;

  if (str == "PARAM_NUM_PARTICLES")
      return Tracking::PARAM_NUM_PARTICLES;
  if (str == "PARAM_SIGMA_ROT")
      return Tracking::PARAM_SIGMA_ROT;
  if (str == "PARAM_SIGMA_TRANSL")
      return Tracking::PARAM_SIGMA_TRANSL;
  if (str == "PARAM_PSIGMA_ROT")
      return Tracking::PARAM_SIGMA_ROT;
  if (str == "PARAM_PSIGMA_TRANSL")
      return Tracking::PARAM_PSIGMA_TRANSL;
  if (str == "PARAM_ISIGMA_ROT")
      return Tracking::PARAM_ISIGMA_ROT;
  if (str == "PARAM_ISIGMA_TRANSL")
      return Tracking::PARAM_ISIGMA_TRANSL;

  if (str == "PARAM_W_SIGMA_ROT")
      return Tracking::PARAM_W_SIGMA_ROT;
  if (str == "PARAM_W_SIGMA_TRANSL")
      return Tracking::PARAM_W_SIGMA_TRANSL;
  if (str == "PARAM_W_PSIGMA_ROT")
      return Tracking::PARAM_W_PSIGMA_ROT;
  if (str == "PARAM_W_PSIGMA_TRANSL")
      return Tracking::PARAM_W_PSIGMA_TRANSL;
  if (str == "PARAM_W_ISIGMA_ROT")
      return Tracking::PARAM_W_ISIGMA_ROT;
  if (str == "PARAM_W_ISIGMA_TRANSL")
      return Tracking::PARAM_W_ISIGMA_TRANSL;

  throw std::string("Unknown parameter: " + str);
}

static std::string ParameterToString(const Tracking::Parameters par)
{
  if (par == Tracking::PARAM_NONE)
      return "PARAM_NONE";

  if (par == Tracking::PARAM_TRANSLATION_LIMIT)
      return "PARAM_TRANSLATION_LIMIT";
  if (par == Tracking::PARAM_TRANSLATION_DYN_LIMIT)
      return "PARAM_TRANSLATION_DYN_LIMIT";
  if (par == Tracking::PARAM_ROTATION_LIMIT)
      return "PARAM_ROTATION_LIMIT";
  if (par == Tracking::PARAM_ROTATION_DYN_LIMIT)
      return "PARAM_ROTATION_DYN_LIMIT";
  if (par == Tracking::PARAM_ROTATION_CURV_LIMIT)
      return "PARAM_ROTATION_CURV_LIMIT";
  if (par == Tracking::PARAM_ROTATION_DYN2_LIMIT)
      return "PARAM_ROTATION_DYN2_LIMIT";
  if (par == Tracking::PARAM_ROTATION_CURV2_LIMIT)
      return "PARAM_ROTATION_CURV2_LIMIT";
  if (par == Tracking::PARAM_TRANSLATION_DYN2_LIMIT)
      return "PARAM_TRANSLATION_DYN2_LIMIT";
  if (par == Tracking::PARAM_ALPHA)
      return "PARAM_ALPHA";
  if (par == Tracking::PARAM_MULT)
      return "PARAM_MULT";

  if (par == Tracking::PARAM_NUM_PARTICLES)
      return "PARAM_NUM_PARTICLES";
  if (par == Tracking::PARAM_SIGMA_ROT)
      return "PARAM_SIGMA_ROT";
  if (par == Tracking::PARAM_SIGMA_TRANSL)
      return "PARAM_SIGMA_TRANSL";
  if (par == Tracking::PARAM_PSIGMA_ROT)
      return "PARAM_SIGMA_ROT";
  if (par == Tracking::PARAM_PSIGMA_TRANSL)
      return "PARAM_PSIGMA_TRANSL";
  if (par == Tracking::PARAM_ISIGMA_ROT)
      return "PARAM_ISIGMA_ROT";
  if (par == Tracking::PARAM_ISIGMA_TRANSL)
      return "PARAM_ISIGMA_TRANSL";

  if (par == Tracking::PARAM_W_SIGMA_ROT)
      return "PARAM_W_SIGMA_ROT";
  if (par == Tracking::PARAM_W_SIGMA_TRANSL)
      return "PARAM_W_SIGMA_TRANSL";
  if (par == Tracking::PARAM_W_PSIGMA_ROT)
      return "PARAM_W_PSIGMA_ROT";
  if (par == Tracking::PARAM_W_PSIGMA_TRANSL)
      return "PARAM_W_PSIGMA_TRANSL";
  if (par == Tracking::PARAM_W_ISIGMA_ROT)
      return "PARAM_W_ISIGMA_ROT";
  if (par == Tracking::PARAM_W_ISIGMA_TRANSL)
      return "PARAM_W_ISIGMA_TRANSL";

  throw std::string("Unknown parameter: " + std::to_string(par));
}

#endif // TRACKING_PARSER_UTILS_H
