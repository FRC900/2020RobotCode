#pragma once
#include <Eigen/Dense>
#include "pf_localization/beacon.hpp"
#include "pf_localization/particle.hpp"
#include "pf_localization/wall.hpp"

// Holds a description of the field.  The map
// includes beacon positions and wall information
class FieldMap
{
	public:
		FieldMap(const Walls &walls, const Beacons &beacons);

		bool outsideField(const ParticleState &point) const;
		const Beacon& getBeacon(size_t i) const;
		const Beacons& getBeacons(void) const;
	private:
		Walls walls_;
		Beacons beacons_;
};

