#include <l3_terrain_model_generator/utils/data_manager.h>

namespace l3_terrain_modeling
{
DataManager::DataManager()
{
}

std::string DataManager::printDatabase()
{
  l3::SharedLock lock(instance().mutex_);

  std::string str = "Elements: " + std::to_string(instance().database_.size()) + "\n";
  for (const std::pair<const std::string, DataHandle::Ptr>& p : instance().database_)
    str += "    " + p.first + "   (" + p.second->type() + ")\n";

  return str;
}
}  // namespace l3_terrain_modeling
