#pragma once

#include <vector>
#include <Eigen/Dense>
#include "pf_localization/distance_and_bearing.hpp"
#include "pf_localization/particle.hpp"

class BeaconCoord
{
	public:
		BeaconCoord(const double x, const double y, const double angle);
		double x(void)     const;
		double y(void)     const;
		double angle(void) const;

		// For google test use
		bool operator==(const BeaconCoord &other) const;

		friend std::ostream& operator<<(std::ostream &os, const BeaconCoord &coord);

	private:
		double x_;
		double y_;
		double angle_;
};

// Beacon - holds field-centric x,y,angle coordinates
//         of targets on the field
//         angle is basically the normal of the wall the
//         beacon is located on - gives an easy way to
//         check if the robot would have to look through a
//         wall to see it
class Beacon
{
	public:
		Beacon(void);
		Beacon(const BeaconCoord &state);

		const BeaconCoord &Get(void) const;

		DistanceAndBearing distance(const ParticleState &coord, bool reverse_angle = false) const;

		double robotBearing(const double angle) const;
		friend std::ostream& operator<<(std::ostream &os, const Beacon &beacon);
	private:
		BeaconCoord coord_;
};

class Beacons
{
	public:
		Beacons(void);
		Beacons(const std::vector<Beacon> &beacons);

		void append(const Beacon &beacon);
		void append(const double x, const double y, const double angle);

		const Beacon& operator[](size_t i) const;

		// Mirror the existing beacons around x / y
		// Useful because the field is generally symmetric
		void mirror_x(void);
		void mirror_y(void);
		size_t size(void) const;

		friend std::ostream& operator<<(std::ostream &os, const Beacons &beacons);
	private:
		std::vector<Beacon> beacons_;
};
