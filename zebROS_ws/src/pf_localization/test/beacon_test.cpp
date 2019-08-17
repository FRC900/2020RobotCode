
#include <gtest/gtest.h>

#include <pf_localization/beacon.hpp>

TEST(ParameterTest, ConstructorParams)
{
	const auto b1 = Beacon(BeaconCoord(1,2,0));
	EXPECT_EQ(BeaconCoord(1,2,0), b1.Get());

	const auto b2 = Beacon();
	EXPECT_EQ(BeaconCoord(0,0,0), b2.Get());
}

std::ostream& operator<<(std::ostream& stream, const DistanceAndBearing &db)
{
	return stream << "D: " << db.distance() << " B:" << db.bearing() << std::endl;
}
TEST(ParameterTest, Distance)
{
	const auto b1 = Beacon(BeaconCoord(1,2,0));
	ParticleState p1;
	p1 << b1.Get().x(),b1.Get().y(),0,0,0,0;
	EXPECT_EQ(DistanceAndBearing(0,0), b1.distance(p1, false));

	p1 << b1.Get().x(),b1.Get().y()+1,0,0,0,0;
	EXPECT_EQ(DistanceAndBearing(1,-M_PI/2.0), b1.distance(p1, false));
	p1 << b1.Get().x()+1,b1.Get().y(),0,0,0,0;
	EXPECT_EQ(DistanceAndBearing(1,M_PI), b1.distance(p1, false));

	p1 << b1.Get().x(),b1.Get().y()+1,0,0,0,0;
	EXPECT_EQ(DistanceAndBearing(1,M_PI/2.0), b1.distance(p1, true));
	p1 << b1.Get().x()+1,b1.Get().y(),0,0,0,0;
	EXPECT_EQ(DistanceAndBearing(1,0), b1.distance(p1, true));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
