// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SAMPLE_MOVEMENT_UR3E_SIM__VISIBILITY_CONTROL_H_
#define SAMPLE_MOVEMENT_UR3E_SIM__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SAMPLE_MOVEMENT_UR3E_SIM_EXPORT __attribute__ ((dllexport))
    #define SAMPLE_MOVEMENT_UR3E_SIM_IMPORT __attribute__ ((dllimport))
  #else
    #define SAMPLE_MOVEMENT_UR3E_SIM_EXPORT __declspec(dllexport)
    #define SAMPLE_MOVEMENT_UR3E_SIM_IMPORT __declspec(dllimport)
  #endif
  #ifdef SAMPLE_MOVEMENT_UR3E_SIM_BUILDING_DLL
    #define SAMPLE_MOVEMENT_UR3E_SIM_PUBLIC SAMPLE_MOVEMENT_UR3E_SIM_EXPORT
  #else
    #define SAMPLE_MOVEMENT_UR3E_SIM_PUBLIC SAMPLE_MOVEMENT_UR3E_SIM_IMPORT
  #endif
  #define SAMPLE_MOVEMENT_UR3E_SIM_PUBLIC_TYPE SAMPLE_MOVEMENT_UR3E_SIM_PUBLIC
  #define SAMPLE_MOVEMENT_UR3E_SIM_LOCAL
#else
  #define SAMPLE_MOVEMENT_UR3E_SIM_EXPORT __attribute__ ((visibility("default")))
  #define SAMPLE_MOVEMENT_UR3E_SIM_IMPORT
  #if __GNUC__ >= 4
    #define SAMPLE_MOVEMENT_UR3E_SIM_PUBLIC __attribute__ ((visibility("default")))
    #define SAMPLE_MOVEMENT_UR3E_SIM_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SAMPLE_MOVEMENT_UR3E_SIM_PUBLIC
    #define SAMPLE_MOVEMENT_UR3E_SIM_LOCAL
  #endif
  #define SAMPLE_MOVEMENT_UR3E_SIM_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // SAMPLE_MOVEMENT_UR3E_SIM__VISIBILITY_CONTROL_H_