#include "pf_localization/field_map.hpp"
FieldMap::FieldMap(const Walls &walls, const Beacons &beacons)
	: walls_(walls)
    , beacons_(beacons)
{
}

bool FieldMap::outsideField(const ParticleState &point) const
{
	return walls_.outsideField(point);
}

const Beacon& FieldMap::getBeacon(size_t i) const
{
	return beacons_[i];
}

const Beacons& FieldMap::getBeacons(void) const
{
	return beacons_;
}

