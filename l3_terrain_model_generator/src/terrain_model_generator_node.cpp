#include <l3_terrain_model_generator/terrain_model_generator_node.h>

#include <pcl_conversions/pcl_conversions.h>

namespace l3_terrain_modeling
{
TerrainModelGeneratorNode::TerrainModelGeneratorNode(ros::NodeHandle& nh)
  : terrain_model_generator_(nh)
{
  // subscribe topics
  reset_terrain_model_sub_ = nh.subscribe("reset", 1, &TerrainModelGeneratorNode::resetCb, this);
  sys_command_sub_ = nh.subscribe("/syscommand", 1, &TerrainModelGeneratorNode::sysCommandCb, this);

  // advertise services
  reset_service_ = nh.advertiseService("reset", &TerrainModelGeneratorNode::resetService, this);
}

void TerrainModelGeneratorNode::loadTestPointCloud(const std::string& path)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>());

  ROS_INFO("Loading point cloud from %s...", path.c_str());
  if (pcl::io::loadPCDFile(path, *point_cloud))
  {
    ROS_ERROR("FAILED!");
    return;
  }
  else
    ROS_INFO("Done!");

  sensor_msgs::PointCloud2 point_cloud_msg;
  pcl::toROSMsg(*point_cloud, point_cloud_msg);
  // point_cloud_msg.header.frame_id = terrain_model_generator_.frameId();
  point_cloud_msg.header.stamp = ros::Time::now();

  ROS_INFO("Generating terrain model...");
  // setPointCloud(point_cloud_msg);
  ROS_INFO("Done!");
}

void TerrainModelGeneratorNode::reset()
{
  ROS_INFO("[TerrainModelGeneratorNode] Reset called!");
  terrain_model_generator_.reset();
}

void TerrainModelGeneratorNode::resetCb(const std_msgs::Empty::ConstPtr /*empty*/)
{
  reset();
}

void TerrainModelGeneratorNode::sysCommandCb(const std_msgs::String::ConstPtr command)
{
  if (command->data == "clear_mapping")
    reset();
}

bool TerrainModelGeneratorNode::resetService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  reset();
  return true;
}
}  // namespace l3_terrain_modeling

int main(int argc, char** argv)
{
  ros::init(argc, argv, "l3_terrain_model");

  ros::NodeHandle nh;
  l3_terrain_modeling::TerrainModelGeneratorNode terrain_model_generator_node(nh);

  for (int i = 1; i < argc; ++i)
  {
    if (std::string(argv[i]) == "-loadTestCloud")
      terrain_model_generator_node.loadTestPointCloud(nh.param("pcl_file", std::string()));
  }

  ros::spin();

  return 0;
}
