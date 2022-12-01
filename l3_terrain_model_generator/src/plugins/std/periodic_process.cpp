#include <l3_terrain_model_generator/plugins/std/periodic_process.h>

namespace l3_terrain_modeling
{
PeriodicProcess::PeriodicProcess()
  : ProcessorPlugin("periodic_process")
{}

bool PeriodicProcess::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!ProcessorPlugin::initialize(params))
    return false;

  double rate = param("rate", 0.0, true);
  timer_ = nh_.createTimer(ros::Rate(rate), &PeriodicProcess::timerCb, this);

  return true;
}

void PeriodicProcess::timerCb(const ros::TimerEvent& event)
{
  UpdatedHandles handles;
  process(Timer(event), handles);
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::PeriodicProcess, l3_terrain_modeling::ProcessorPlugin)
