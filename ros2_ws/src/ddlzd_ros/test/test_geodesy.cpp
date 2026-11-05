#include "ddlzd_ros/geodesy.hpp"

#include <GeographicLib/Geodesic.hpp>
#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

TEST(Geodesy, ZeroOffsetPreservesOrigin)
{
  const auto result = ddlzd_ros::offsetEnuToWgs84(48.0764, 11.6383, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(result.latitude, 48.0764);
  EXPECT_DOUBLE_EQ(result.longitude, 11.6383);
}

TEST(Geodesy, EastNorthOffsetRoundTripsOnWgs84)
{
  constexpr double origin_lat = 48.0764;
  constexpr double origin_lon = 11.6383;
  constexpr double east = 30.0;
  constexpr double north = 40.0;
  const auto target = ddlzd_ros::offsetEnuToWgs84(origin_lat, origin_lon, east, north);

  double distance = 0.0;
  double forward_azimuth = 0.0;
  double reverse_azimuth = 0.0;
  GeographicLib::Geodesic::WGS84().Inverse(
    origin_lat, origin_lon, target.latitude, target.longitude,
    distance, forward_azimuth, reverse_azimuth);
  EXPECT_NEAR(distance, 50.0, 1e-6);
  EXPECT_NEAR(
    forward_azimuth,
    std::atan2(east, north) * 180.0 / 3.14159265358979323846,
    1e-9);
}

TEST(Geodesy, RejectsInvalidLatitude)
{
  EXPECT_THROW(
    static_cast<void>(ddlzd_ros::offsetEnuToWgs84(91.0, 0.0, 1.0, 1.0)),
    std::invalid_argument);
}
