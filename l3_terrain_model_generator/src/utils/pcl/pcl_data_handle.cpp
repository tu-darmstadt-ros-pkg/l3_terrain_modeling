#include <l3_terrain_model_generator/utils/pcl/pcl_data_handle.h>

namespace l3_terrain_modeling
{
PointType dispatchPointType(const std::string& point_type)
{
  if (point_type == "PointXYZ")
    return PointType::PointXYZ;
  else if (point_type == "PointXYZI")
    return PointType::PointXYZI;
  else if (point_type == "PointXYZL")
    return PointType::PointXYZL;
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
    case PointType::PointXYZI:
      return "PointXYZI";
    case PointType::PointXYZL:
      return "PointXYZL";
    case PointType::PointXYZRGB:
      return "PointXYZRGB";
    case PointType::PointNormal:
      return "PointNormal";
    default:
      return "Invalid";
  }
}
}  // namespace l3_terrain_modeling
