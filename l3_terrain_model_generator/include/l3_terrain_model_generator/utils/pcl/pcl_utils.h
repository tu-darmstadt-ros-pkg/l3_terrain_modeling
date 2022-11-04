//=================================================================================================
// Copyright (c) 2022, Alexander Stumpf, TU Darmstadt
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

#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/for_each_type.h>
#include <pcl/PolygonMesh.h>

#include <l3_libs/types/types.h>

namespace l3_terrain_modeling
{
/**
 * @brief Converts a PCL PolygonMesh into a marker in order to visualize the mesh in RViz.
 * @param mesh pcl-based mesh
 * @param marker marker to be published for RViz
 * @param color homogenous mesh color
 * @param consistent_vertex_ordering ifset to true, the input mesh's vertices of each facet are consistently ordered (normals are pointing in the same direction)
 */
void fromPCL(const pcl::PolygonMesh& mesh, visualization_msgs::Marker& marker, const std_msgs::ColorRGBA& color, bool consistent_vertex_ordering = false);

template <typename PointT>
PointT createNanPoint()
{
  // generate point with nan fields
  PointT p;
  using FieldList = typename pcl::traits::fieldList<PointT>::type;
  pcl::for_each_type<FieldList>(pcl::SetIfFieldExists<PointT, float>(p, "x", std::numeric_limits<float>::quiet_NaN()));
  pcl::for_each_type<FieldList>(pcl::SetIfFieldExists<PointT, float>(p, "y", std::numeric_limits<float>::quiet_NaN()));
  pcl::for_each_type<FieldList>(pcl::SetIfFieldExists<PointT, float>(p, "z", std::numeric_limits<float>::quiet_NaN()));
  return p;
}

/**
 * @brief Retrieves point color if available
 * @return point color, if no color is available all values are set to zero.
 */
template <typename PointT, typename std::enable_if<std::is_same<PointT, pcl::PointXYZRGB>::value>::type* = nullptr>
pcl::RGB getPointColor(const PointT& p)
{
#ifdef POINTXYZI_SUPPORT
  return pcl::RGB(p.r, p.g, p.b);
#else
  pcl::RGB color;
  color.r = p.r;
  color.g = p.g;
  color.b = p.b;
  color.a = 255;
  return color;
#endif
}

template <typename PointT, typename std::enable_if<!std::is_same<PointT, pcl::PointXYZRGB>::value>::type* = nullptr>
pcl::RGB getPointColor(const PointT& p)
{
  pcl::RGB color;
  color.a = 0;
  return color;
}

template <typename PointT>
void getPointCloudBoundary(const typename pcl::PointCloud<PointT>::ConstPtr cloud, l3::Vector3& min, l3::Vector3& max)
{
  min.x() = min.y() = min.z() = std::numeric_limits<double>::max();
  max.x() = max.y() = max.z() = -std::numeric_limits<double>::max();

  for (typename pcl::PointCloud<PointT>::const_iterator itr = cloud->begin(); itr != cloud->end(); itr++)
  {
    const PointT& p = *itr;
    min.x() = std::min(min.x(), static_cast<double>(p.x));
    min.y() = std::min(min.y(), static_cast<double>(p.y));
    min.z() = std::min(min.z(), static_cast<double>(p.z));
    max.x() = std::max(max.x(), static_cast<double>(p.x));
    max.y() = std::max(max.y(), static_cast<double>(p.y));
    max.z() = std::max(max.z(), static_cast<double>(p.z));
  }
}
}  // namespace l3_terrain_modeling
