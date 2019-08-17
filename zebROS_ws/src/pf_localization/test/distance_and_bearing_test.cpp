#include <ros/ros.h>

#include <gtest/gtest.h>

#include <pf_localization/distance_and_bearing.hpp>

TEST(ParameterTest, ConstructorParams)
{
  RecordProperty("description","This test verifies constructor parameters are read and stored correctly");

  auto d_and_b1 = DistanceAndBearing(1.0, 2.0);
  EXPECT_EQ(1.0, d_and_b1.distance());
  EXPECT_EQ(2.0, d_and_b1.bearing());

  auto d_and_b2 = DistanceAndBearing(10.0, -20.0);
  EXPECT_EQ(10.0,  d_and_b2.distance());
  EXPECT_EQ(-20.0, d_and_b2.bearing());
}

TEST(ParameterTest, OperatorMinus)
{
  RecordProperty("description","This test verifies constructor parameters are read and stored correctly");
  auto d_and_b1 = DistanceAndBearing(1.0, 2.0);
  d_and_b1 -= d_and_b1;
  EXPECT_EQ(0.0, d_and_b1.distance());
  EXPECT_EQ(0.0, d_and_b1.bearing());

  auto d_and_b2 = DistanceAndBearing(10.0, -20.0);
  auto d_and_b3 = DistanceAndBearing( 1.0, -1.0);
  d_and_b2 -= d_and_b3;
  EXPECT_EQ(9.0, d_and_b2.distance());
  EXPECT_EQ(-19.0, d_and_b2.bearing());
  EXPECT_EQ(1.0, d_and_b3.distance());
  EXPECT_EQ(-1.0, d_and_b3.bearing());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
