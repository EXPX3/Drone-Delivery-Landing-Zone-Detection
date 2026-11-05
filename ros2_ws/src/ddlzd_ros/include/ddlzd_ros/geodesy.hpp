#pragma once

namespace ddlzd_ros
{

struct Wgs84Coordinate
{
  double latitude;
  double longitude;
};

[[nodiscard]] Wgs84Coordinate offsetEnuToWgs84(
  double latitude_deg, double longitude_deg, double east_m, double north_m);

}  // namespace ddlzd_ros
