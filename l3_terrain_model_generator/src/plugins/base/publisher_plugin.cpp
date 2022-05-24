#include <l3_terrain_model_generator/plugins/base/publisher_plugin.h>

namespace l3_terrain_modeling
{
PublisherPlugin::PublisherPlugin(const std::string& name)
  : ProcessPlugin(name)
{
}

bool PublisherPlugin::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!ProcessPlugin::initialize(params))
    return false;

  latch_topics_ = param("latch", true, true);
  initial_publish_ = latch_topics_;

  return true;
}
}  // namespace l3_terrain_modeling
