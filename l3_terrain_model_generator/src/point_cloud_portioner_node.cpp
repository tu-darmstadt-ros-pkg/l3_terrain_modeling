#include <l3_terrain_model_generator/point_cloud_portioner_node.h>

#include <pcl_conversions/pcl_conversions.h>

namespace l3_terrain_modeling
{
PointCloudPortionerNode::PointCloudPortionerNode(ros::NodeHandle& nh)
{
  ros::NodeHandle pnh("~");

  double publish_rate;

  pnh.param("sensor_frame_id", sensor_frame_id_, std::string("lidar"));
  pnh.param("publish_rate", publish_rate, 30.0);
  pnh.param("points_per_update", (int&)points_per_update_, 500);
  pnh.param("play_in_loop", play_in_loop_, true);

  // publish topics
  point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_update", 1);

  if (!pnh.hasParam("pcl_file"))
  {
    ROS_ERROR("No PCL file given. Please start node with argument 'pcl_file:=<path to file>'");
    exit(0);
  }

  loadPointCloud(pnh.param("pcl_file", std::string()));

  publish_timer_ = nh.createTimer(ros::Duration(1.0 / publish_rate), &PointCloudPortionerNode::update, this);
}

PointCloudPortionerNode::~PointCloudPortionerNode() {}

void PointCloudPortionerNode::loadPointCloud(const std::string& path)
{
  //  l3_terrain_modeling::TerrainClassifierParams params;
  //  terrain_classifier->getParams(params);
  //  params.ed_radius = 0.05;
  //  params.ed_max_std = 0.02;
  //  params.ed_non_max_supp_radius = 0.05;
  //  params.gg_res = 0.02;
  //  terrain_classifier->setParams(params);

  ROS_INFO("Loading point cloud from %s...", path.c_str());
  if (pcl::io::loadPCDFile(path, point_cloud_))
  {
    ROS_ERROR("FAILED!");
    exit(0);
  }
  else
    ROS_INFO("Done!");

  //pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(point_cloud_);
  //filterVoxelGrid<pcl::PointXYZ>(point_cloud, 0.02, 0.02, 2.0);
  //terrain_model_generator_.filterPointCloudData<pcl::PointXYZ>(point_cloud);
  //point_cloud_ = *point_cloud;

  current_point_index_ = 0;
}

void PointCloudPortionerNode::update(const ros::TimerEvent& /*event*/)
{
  pcl::PointCloud<pcl::PointXYZ> point_cloud_portion;
  sensor_msgs::PointCloud2 point_cloud_msg;

  if (point_cloud_.size() <= current_point_index_)
  {
    if (play_in_loop_)
      current_point_index_ = (current_point_index_ + points_per_update_) % point_cloud_.size();
    else
      return;
  }

  int offset = current_point_index_;
  for (unsigned int i = 0; i < points_per_update_; i++)
  {
    if (point_cloud_.size() <= offset + i)
    {
      if (play_in_loop_)
        offset = -i;
      else
        break;
    }

    point_cloud_portion.push_back(point_cloud_[offset + i]);
  }

  current_point_index_ += points_per_update_;

  if (point_cloud_pub_.getNumSubscribers() > 0)
  {
    pcl::toROSMsg(point_cloud_portion, point_cloud_msg);
    point_cloud_msg.header.stamp = ros::Time::now();
    point_cloud_msg.header.frame_id = sensor_frame_id_;
    point_cloud_pub_.publish(point_cloud_msg);
  }
}
}  // namespace l3_terrain_modeling

int main(int argc, char** argv)
{
  ros::init(argc, argv, "l3_point_cloud_portioner");

  ros::NodeHandle nh;
  l3_terrain_modeling::PointCloudPortionerNode l3_point_cloud_portioner(nh);

  ros::spin();

  return 0;
}
