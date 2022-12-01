#include <l3_terrain_model_generator/terrain_model_generator.h>

#include <vigir_pluginlib/plugin_manager.h>

#include <l3_libs/conversions/l3_msg_std_conversions.h>
#include <l3_libs/conversions/l3_tf_conversions.h>

#include <l3_terrain_model_generator/typedefs.h>

namespace l3_terrain_modeling
{
TerrainModelGenerator::TerrainModelGenerator(ros::NodeHandle& nh)
  : sensors_("TerrainModelGenerator::Sensors")
  , processes_("TerrainModelGenerator::Processes")
{
  ros::NodeHandle pnh("~");

  // initialize plugin manager
  vigir_pluginlib::PluginManager::initialize(nh);

  // register class loader
  vigir_pluginlib::PluginManager::addPluginClassLoader<SensorPlugin>("l3_terrain_model_generator", "l3_terrain_modeling::SensorPlugin");
  vigir_pluginlib::PluginManager::addPluginClassLoader<ProcessorPlugin>("l3_terrain_model_generator", "l3_terrain_modeling::ProcessorPlugin");

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

  sensors_.loadPlugins();
  processes_.loadPlugins();

  return true;
}

void TerrainModelGenerator::reset()
{
  sensors_.reset();
  processes_.call([](ProcessorPlugin::Ptr process) { process->reset(); });
}
}  // namespace l3_terrain_modeling
