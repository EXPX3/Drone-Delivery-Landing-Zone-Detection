#include "ddlzd_ros/geodesy.hpp"

#include <GeographicLib/Geodesic.hpp>

#include <cmath>
#include <stdexcept>

namespace ddlzd_ros
{

Wgs84Coordinate offsetEnuToWgs84(
  const double latitude_deg, const double longitude_deg,
  const double east_m, const double north_m)
{
  if (!std::isfinite(latitude_deg) || latitude_deg < -90.0 || latitude_deg > 90.0 ||
    !std::isfinite(longitude_deg) || longitude_deg < -180.0 || longitude_deg > 180.0 ||
    !std::isfinite(east_m) || !std::isfinite(north_m))
  {
    throw std::invalid_argument("invalid WGS84 origin or ENU offset");
  }

  const double distance_m = std::hypot(east_m, north_m);
  if (distance_m == 0.0) {
    return Wgs84Coordinate{latitude_deg, longitude_deg};
  }
  const double azimuth_deg = std::atan2(east_m, north_m) * 180.0 /
    3.14159265358979323846;
  double target_latitude = 0.0;
  double target_longitude = 0.0;
  GeographicLib::Geodesic::WGS84().Direct(
    latitude_deg, longitude_deg, azimuth_deg, distance_m,
    target_latitude, target_longitude);
  return Wgs84Coordinate{target_latitude, target_longitude};
}

}  // namespace ddlzd_ros
