#include <l3_terrain_model_generator/plugins/base/filter_plugin.h>

#include <l3_libs/conversions/l3_msg_std_conversions.h>

namespace l3_terrain_modeling
{
FilterPlugin::FilterPlugin(const std::string& name)
  : ProcessPlugin(name)
  , tf_listener_(tf_buffer_)
{}
}  // namespace l3_terrain_modeling
