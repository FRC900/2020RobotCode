#pragma once
#include <ostream>
// Basic container class holding a distance and bearing between
// two objects. Mostly used for measuring between the proposed
// robot position and detections and/or beacons.
class DistanceAndBearing
{
	public:
		DistanceAndBearing(const double distance, const double bearing);
		DistanceAndBearing& operator-=(const DistanceAndBearing &rhs);
		bool operator==(const DistanceAndBearing &other) const;

		double distance(void) const;
		double bearing(void) const;

		friend std::ostream& operator<<(std::ostream &os, const DistanceAndBearing &db);

	private:
		double distance_;
		double bearing_;
};


