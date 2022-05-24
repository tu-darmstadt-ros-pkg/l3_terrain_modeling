#include <l3_terrain_model_generator/utils/pcl/pcl_utils.h>

#include <l3_libs/conversions/l3_msg_std_conversions.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

namespace l3_terrain_modeling
{
void fromPCL(const pcl::PolygonMesh& mesh, visualization_msgs::Marker& marker, const std_msgs::ColorRGBA& color, bool consistent_vertex_ordering)
{
  marker.id = 0;
  marker.ns = "mesh";
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  marker.lifetime = ros::Duration(0.0);
  marker.pose.position.x = marker.pose.position.y = marker.pose.position.z = 0.0;
  marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
  marker.color = color;

  marker.header = pcl_conversions::fromPCL(mesh.header);

  // conversion of vertices
  pcl::PointCloud<pcl::PointXYZ> vertices;
  pcl::fromPCLPointCloud2(mesh.cloud, vertices);

  // convert each vertex
  for (const pcl::Vertices& v : mesh.polygons)
  {
    l3::PointArray facet;
    facet.resize(3);

    int i = 0;
    for (const uint32_t& idx : v.vertices)
    {
      const pcl::PointXYZ& v = vertices.points[idx];
      facet[i].setX(v.x);
      facet[i].setY(v.y);
      facet[i].setZ(v.z);

      if (consistent_vertex_ordering)
      {
        geometry_msgs::Point p;
        l3::pointL3ToMsg(facet[0], p);
        marker.points.push_back(p);
      }
      else
      {
        // sort points of each facet to get correct facing
        if (++i == 3)
        {
          // determine the normal of the vertex
          l3::Vector3 n = (facet[0] - facet[1]).cross(facet[1] - facet[2]);

          if (n.z() > 0.0)
          {
            geometry_msgs::Point p;
            l3::pointL3ToMsg(facet[0], p);
            marker.points.push_back(p);
            l3::pointL3ToMsg(facet[1], p);
            marker.points.push_back(p);
            l3::pointL3ToMsg(facet[2], p);
            marker.points.push_back(p);
          }
          else
          {
            geometry_msgs::Point p;
            l3::pointL3ToMsg(facet[0], p);
            marker.points.push_back(p);
            l3::pointL3ToMsg(facet[2], p);
            marker.points.push_back(p);
            l3::pointL3ToMsg(facet[1], p);
            marker.points.push_back(p);
          }
        }

        i = 0;
      }
    }

    if (i != 0)
      ROS_WARN("Input mesh does not contain a multiple of 3 edges. Conversion from pcl to marker may be incorrect.");
  }
}
}  // namespace l3_terrain_modeling
