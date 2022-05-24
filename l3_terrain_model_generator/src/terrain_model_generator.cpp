#include <l3_terrain_model_generator/terrain_model_generator.h>

#include <vigir_pluginlib/plugin_manager.h>

#include <l3_libs/conversions/l3_msg_std_conversions.h>
#include <l3_libs/conversions/l3_tf_conversions.h>

#include <l3_terrain_model_generator/typedefs.h>

namespace l3_terrain_modeling
{
TerrainModelGenerator::TerrainModelGenerator(ros::NodeHandle& nh)
{
  ros::NodeHandle pnh("~");

  // initialize plugin manager
  vigir_pluginlib::PluginManager::initialize(nh);

  // register class loader
  vigir_pluginlib::PluginManager::addPluginClassLoader<SensorPlugin>("l3_terrain_model_generator", "l3_terrain_modeling::SensorPlugin");
  vigir_pluginlib::PluginManager::addPluginClassLoader<ProcessPlugin>("l3_terrain_model_generator", "l3_terrain_modeling::ProcessPlugin");

  // load plugin set
  loadPluginSet(pnh.param("plugin_set", std::string("example")));

  ROS_INFO("\n[Database]\n%s", DataManager::printDatabase().c_str());
}

bool TerrainModelGenerator::loadPluginSet(const std::string& name)
{
  if (!vigir_pluginlib::PluginManager::loadPluginSet(name))
  {
    ROS_ERROR("[TerrainModelGenerator] Failed to load plugins!");
    return false;
  }

  sensor_.loadPlugins();
  process_.loadPlugins();

  return true;
}

void TerrainModelGenerator::reset()
{
  sensor_.reset();
  process_.call([](ProcessPlugin::Ptr process) { process->reset(); });
}
}  // namespace l3_terrain_modeling
