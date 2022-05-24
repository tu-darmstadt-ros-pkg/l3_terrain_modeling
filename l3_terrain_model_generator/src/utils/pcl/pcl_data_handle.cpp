#include <l3_terrain_model_generator/utils/pcl/pcl_data_handle.h>

namespace l3_terrain_modeling
{
PointType dispatchPointType(const std::string& point_type)
{
  if (point_type == "PointXYZ")
    return PointType::PointXYZ;
#ifdef POINTXYZI_SUPPORT
  else if (point_type == "PointXYZI")
    return PointType::PointXYZI;
#endif
  else if (point_type == "PointXYZRGB")
    return PointType::PointXYZRGB;
  else if (point_type == "PointNormal")
    return PointType::PointNormal;

  return PointType::Invalid;
}

std::string toString(const PointType& point_type)
{
  switch (point_type)
  {
    case PointType::PointXYZ:
      return "PointXYZ";
#ifdef POINTXYZI_SUPPORT
    case PointType::PointXYZI:
      return "PointXYZI";
#endif
    case PointType::PointXYZRGB:
      return "PointXYZRGB";
    case PointType::PointNormal:
      return "PointNormal";
    default:
      return "Invalid";
  }
}
}  // namespace l3_terrain_modeling
