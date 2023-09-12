//=================================================================================================
// Copyright (c) 2023, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#pragma once

#include <l3_terrain_model_generator/typedefs.h>
#include <l3_terrain_model_generator/utils/data_manager.h>
#include <l3_terrain_model_generator/utils/utils.h>

/* Helper macros to get handle to exsiting data. */
#define GET_INPUT_HANDLE(Type, ParamKey, DefaultHandleName, Handle)                                                    \
  {                                                                                                                    \
    const std::string& input_data_name = l3_terrain_modeling::getInputDataParam(getParams(), std::string((ParamKey)),  \
                                                                                std::string((DefaultHandleName)));     \
    (Handle) = DataManager::getHandle<Type>(this, input_data_name);                                                    \
    if (!(Handle))                                                                                                     \
    {                                                                                                                  \
      ROS_ERROR("[%s] Cannot get input data handle \"%s\" of type \"%s\"!", getName().c_str(),                         \
                input_data_name.c_str(), l3::getTypeName<Type>().c_str());                                             \
      return false;                                                                                                    \
    }                                                                                                                  \
  }

#define GET_INPUT_HANDLE_DEFAULT(Type, DefaultHandleName, Handle)                                                      \
  GET_INPUT_HANDLE(Type, "input_data", DefaultHandleName, Handle)

/* Helper macros to get pcl handle to exsiting data. */
#define GET_INPUT_PCL_HANDLE(ParamKey, DefaultHandleName, DefaultPointType, Handle)                                    \
  {                                                                                                                    \
    const std::string& input_data_name =                                                                               \
        l3_terrain_modeling::getInputDataParam(getParams(), (ParamKey), std::string((DefaultHandleName)));             \
    const std::string& point_type = l3_terrain_modeling::getInputDataParam(                                            \
        getParams(), std::string((ParamKey)) + "_point_type", std::string((DefaultPointType)));                        \
    (Handle) = PclDataHandle<pcl::PointCloud>::makeHandle(this, input_data_name, point_type);                          \
    if (!(Handle))                                                                                                     \
    {                                                                                                                  \
      ROS_ERROR("[%s] Data handle \"%s\" seems not to contain valid pcl data!", getName().c_str(),                     \
                input_data_name.c_str());                                                                              \
      return false;                                                                                                    \
    }                                                                                                                  \
  }

#define GET_INPUT_PCL_HANDLE_DEFAULT(DefaultHandleName, Handle)                                                        \
  GET_INPUT_PCL_HANDLE("input_data", DefaultHandleName, "PointXYZ", Handle)

/* Helper macros to get pcl handle to exsiting data. */
#define GET_INPUT_TYPED_PCL_HANDLE(ParamKey, DefaultHandleName, PointType, Handle)                                     \
  {                                                                                                                    \
    const std::string& input_data_name =                                                                               \
        l3_terrain_modeling::getInputDataParam(getParams(), (ParamKey), std::string((DefaultHandleName)));             \
    (Handle) = PclDataHandle<pcl::PointCloud>::makeHandle(this, input_data_name, PointType);                           \
    if (!(Handle))                                                                                                     \
    {                                                                                                                  \
      ROS_ERROR("[%s] Data handle \"%s\" seems not to contain valid pcl data!", getName().c_str(),                     \
                input_data_name.c_str());                                                                              \
      return false;                                                                                                    \
    }                                                                                                                  \
  }

#define GET_INPUT_TYPED_PCL_HANDLE_DEFAULT(DefaultHandleName, PointType, Handle)                                       \
  GET_INPUT_TYPED_PCL_HANDLE("input_data", DefaultHandleName, PointType, Handle)

/* Helper macros to create new handle. */
#define GET_OUTPUT_HANDLE(Data, ParamKey, DefaultHandleName, Handle)                                                   \
  {                                                                                                                    \
    const std::string& output_data_name =                                                                              \
        l3_terrain_modeling::getOutputDataParam(getParams(), (ParamKey), std::string((DefaultHandleName)));            \
    (Handle) = DataManager::addData(this, output_data_name, std::move(Data));                                          \
    if (!(Handle))                                                                                                     \
    {                                                                                                                  \
      ROS_ERROR("[%s] Cannot create output data handle \"%s\"!", getName().c_str(), output_data_name.c_str());         \
      return false;                                                                                                    \
    }                                                                                                                  \
  }

#define GET_OUTPUT_HANDLE_DEFAULT(Data, DefaultHandleName, Handle)                                                     \
  GET_OUTPUT_HANDLE(Data, "output_data", DefaultHandleName, Handle)

/* Helper macros to create new pcl handle. */
#define GET_OUTPUT_PCL_HANDLE(ParamKey, DefaultHandleName, DefaultPointType, Handle)                                   \
  {                                                                                                                    \
    const std::string& output_data_name =                                                                              \
        l3_terrain_modeling::getOutputDataParam(getParams(), (ParamKey), std::string((DefaultHandleName)));            \
    const std::string& point_type = l3_terrain_modeling::getOutputDataParam(                                           \
        getParams(), std::string((ParamKey)) + "_point_type", std::string((DefaultPointType)));                        \
    (Handle) = PclDataHandle<pcl::PointCloud>::makeHandle(this, output_data_name, point_type);                         \
    if (!(Handle))                                                                                                     \
    {                                                                                                                  \
      ROS_ERROR("[%s] Cannot create pcl handle. Unsupported point type: \"%s\"", getName().c_str(),                    \
                point_type.c_str());                                                                                   \
      return false;                                                                                                    \
    }                                                                                                                  \
  }

#define GET_OUTPUT_PCL_HANDLE_DEFAULT(DefaultHandleName, Handle)                                                       \
  GET_OUTPUT_PCL_HANDLE("output_data", DefaultHandleName, "PointXYZ", Handle)

/* Helper macros to create new pcl handle. */
#define GET_OUTPUT_TYPED_PCL_HANDLE(ParamKey, DefaultHandleName, PointType, Handle)                                    \
  {                                                                                                                    \
    const std::string& output_data_name =                                                                              \
        l3_terrain_modeling::getOutputDataParam(getParams(), (ParamKey), std::string((DefaultHandleName)));            \
    (Handle) = PclDataHandle<pcl::PointCloud>::makeHandle(this, output_data_name, PointType);                          \
    if (!(Handle))                                                                                                     \
    {                                                                                                                  \
      ROS_ERROR("[%s] Cannot create pcl handle. Unsupported point type: \"%s\"", getName().c_str(),                    \
                point_type.c_str());                                                                                   \
      return false;                                                                                                    \
    }                                                                                                                  \
  }

#define GET_OUTPUT_TYPED_PCL_HANDLE_DEFAULT(DefaultHandleName, PointType, Handle)                                      \
  GET_OUTPUT_TYPED_PCL_HANDLE("output_data", DefaultHandleName, PointType, Handle)
