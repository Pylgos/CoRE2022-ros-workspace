// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef ROBOT_INTERFACE_PROXY__VISIBILITY_H_
#define ROBOT_INTERFACE_PROXY__VISIBILITY_H_

#ifdef __cplusplus
extern "C" {
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

#ifdef __GNUC__
#define ROBOT_INTERFACE_PROXY_EXPORT __attribute__((dllexport))
#define ROBOT_INTERFACE_PROXY_IMPORT __attribute__((dllimport))
#else
#define ROBOT_INTERFACE_PROXY_EXPORT __declspec(dllexport)
#define ROBOT_INTERFACE_PROXY_IMPORT __declspec(dllimport)
#endif

#ifdef ROBOT_INTERFACE_PROXY_DLL
#define ROBOT_INTERFACE_PROXY_PUBLIC ROBOT_INTERFACE_PROXY_EXPORT
#else
#define ROBOT_INTERFACE_PROXY_PUBLIC ROBOT_INTERFACE_PROXY_IMPORT
#endif

#define ROBOT_INTERFACE_PROXY_PUBLIC_TYPE ROBOT_INTERFACE_PROXY_PUBLIC

#define ROBOT_INTERFACE_PROXY_LOCAL

#else

#define ROBOT_INTERFACE_PROXY_EXPORT __attribute__((visibility("default")))
#define ROBOT_INTERFACE_PROXY_IMPORT

#if __GNUC__ >= 4
#define ROBOT_INTERFACE_PROXY_PUBLIC __attribute__((visibility("default")))
#define ROBOT_INTERFACE_PROXY_LOCAL __attribute__((visibility("hidden")))
#else
#define ROBOT_INTERFACE_PROXY_PUBLIC
#define ROBOT_INTERFACE_PROXY_LOCAL
#endif

#define ROBOT_INTERFACE_PROXY_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_INTERFACE_PROXY__VISIBILITY_H_