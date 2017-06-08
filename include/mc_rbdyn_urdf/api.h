/* Copyright 2015-2017 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is part of mc_rbdyn_urdf.
 *
 * mc_rbdyn_urdf is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * mc_rbdyn_urdf is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with mc_rbdyn_urdf.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifdef WIN32
    #define MCRBDYNURDF_DLLIMPORT __declspec(dllimport)
    #define MCRBDYNURDF_DLLEXPORT __declspec(dllexport)
#else
    #define MCRBDYNURDF_DLLIMPORT
    #define MCRBDYNURDF_DLLEXPORT
#endif

#ifdef MCRBDYNURDF_BUILDING
    #define MCRBDYNURDF_API MCRBDYNURDF_DLLEXPORT
#else
    #define MCRBDYNURDF_API MCRBDYNURDF_DLLIMPORT
#endif
