#include <l3_terrain_model_generator/plugins/aggregator/process_chain.h>

namespace l3_terrain_modeling
{
ProcessChain::ProcessChain(const std::string& name)
  : PluginAggregator<ProcessPlugin>(name)
{
}

bool ProcessChain::loadPlugins(const std::vector<std::string>& names, bool print_warning)
{
  plugins_.clear();

  // get plugins
  for (const std::string& name : names)
  {
    ProcessPlugin::Ptr plugin;
    if (vigir_pluginlib::PluginManager::getPlugin(plugin, name))
      plugins_.push_back(plugin);
    else
    {
      ROS_WARN("[%s] loadPlugins: Couldn't find plugin named '%s' of type '%s'.", name_.c_str(), name.c_str(), vigir_pluginlib::TypeClass::get<ProcessPlugin>().c_str());
      return false;
    }
  }

  // check result
  if (plugins_.empty())
  {
    if (print_warning)
      ROS_WARN("[%s] loadPlugins: Couldn't find any plugin of type '%s'.", name_.c_str(), vigir_pluginlib::TypeClass::get<ProcessPlugin>().c_str());
    else
      ROS_INFO("[%s] No chain loaded.", name_.c_str());
  }
  else
  {
    ROS_INFO("[%s] Chain loaded:", name_.c_str());
    for (const ProcessPlugin::Ptr plugin : plugins_)
    {
      if (plugin)
        ROS_INFO("    %s", plugin->getName().c_str());
    }
  }

  return true;
}
}  // namespace l3_terrain_modeling
