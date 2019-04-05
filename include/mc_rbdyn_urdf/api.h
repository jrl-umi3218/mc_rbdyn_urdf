/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#ifdef WIN32
#  define MCRBDYNURDF_DLLIMPORT __declspec(dllimport)
#  define MCRBDYNURDF_DLLEXPORT __declspec(dllexport)
#else
#  define MCRBDYNURDF_DLLIMPORT
#  define MCRBDYNURDF_DLLEXPORT
#endif

#ifdef MCRBDYNURDF_BUILDING
#  define MCRBDYNURDF_API MCRBDYNURDF_DLLEXPORT
#else
#  define MCRBDYNURDF_API MCRBDYNURDF_DLLIMPORT
#endif
