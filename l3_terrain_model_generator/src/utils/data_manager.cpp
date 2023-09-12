#include <l3_terrain_model_generator/utils/data_manager.h>

namespace l3_terrain_modeling
{
DataManager::DataManager()
{
}

std::string DataManager::printDatabase()
{
  l3::SharedLock lock(instance().mutex_);

  // print object database
  std::string str = "Elements: " + std::to_string(instance().database_.size()) + "\n";
  for (const std::pair<const std::string, DataHandle::Ptr>& p : instance().database_)
    str += "    " + p.first + "   (" + p.second->type() + ")\n";

  // print owner database
  // str += "\nOwners: \n";
  // for (const std::pair<const vigir_pluginlib::Plugin*, std::set<DataHandle::ConstPtr>>& p : instance().owner_database_)
  // {
  //   str += "    " + p.first->getName() + "\n";
  //   for (DataHandle::ConstPtr handle : p.second)
  //     str += "        " + handle->name() + "   (" + handle->type() + ")\n";
  // }

  // print handle usage
  str += "\nHandle usage: \n";

  std::unordered_map<const DataHandle*, std::set<const vigir_pluginlib::Plugin*>> handle_usage;
  for (const std::pair<const vigir_pluginlib::Plugin*, std::set<DataHandle::ConstPtr>>& p : instance().owner_database_)
  {
    for (DataHandle::ConstPtr handle : p.second)
      handle_usage[handle.get()].insert(p.first);
  }

  for (const std::pair<const DataHandle*, std::set<const vigir_pluginlib::Plugin*>>& p : handle_usage)
  {
    str += "    " + p.first->name() + "   (" + p.first->type() + ")\n";
    for (const vigir_pluginlib::Plugin* owner : p.second)
      str += "        " + owner->getName() + "\n";
  }

  return str;
}
}  // namespace l3_terrain_modeling
