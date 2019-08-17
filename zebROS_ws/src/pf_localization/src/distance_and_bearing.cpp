#include "pf_localization/distance_and_bearing.hpp"

// Simple container class holding a distance / bearing
// between two coordinates
DistanceAndBearing::DistanceAndBearing(const double distance, const double bearing)
	: distance_(distance)
	, bearing_(bearing)
{
}

DistanceAndBearing& DistanceAndBearing::operator-=(const DistanceAndBearing &rhs)
{
	this->distance_ -= rhs.distance_;
	this->bearing_  -= rhs.bearing_;
	return *this;
}

bool DistanceAndBearing::operator==(const DistanceAndBearing &other) const
{
	return (distance_ == other.distance_) && (bearing_ == other.bearing_);
}

double DistanceAndBearing::distance(void) const
{
	return distance_;
}
double DistanceAndBearing::bearing(void) const
{
	return bearing_;
}

std::ostream& operator<< (std::ostream &stream, const DistanceAndBearing &db)
{
	stream << "DistanceAndBearing : distance=" << db.distance_ << " bearing=" << db.bearing_ << std::endl;
	return stream;
}



