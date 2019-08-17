#include "pf_localization/beacon.hpp"
#include "pf_localization/util.h"

// Simple container class holding x, y and angle coordinates for
// each beacon
BeaconCoord::BeaconCoord(const double x, const double y, const double angle)
	: x_(x)
	, y_(y)
	, angle_(fast_normalize_angle(angle))
{
}

double BeaconCoord::x(void)     const { return x_; }
double BeaconCoord::y(void)     const { return y_; }
double BeaconCoord::angle(void) const { return angle_; }

bool BeaconCoord::operator==(const BeaconCoord &other) const
{
	return (x_ == other.x_) && (y_ == other.y_) && (angle_ == other.angle_);
}

std::ostream& operator<<(std::ostream &stream, const BeaconCoord &coord)
{
	stream << "BeaconCoord : x=" << coord.x_
		<< " y=" << coord.y_ <<
		" angle=" << coord.angle_ << std::endl;
	return stream;
}
std::ostream& operator<<(std::ostream &stream, const Beacon& beacon)
{
	stream << "Beacon : " << std::endl << "\t" << beacon.coord_;
	return stream;
}
std::ostream& operator<<(std::ostream &stream, const Beacons& beacons)
{
	stream << "Beacons : " << std::endl;
	for (size_t i = 0; i < beacons.beacons_.size(); i++)
		stream << beacons.beacons_[i];
	return stream;
}

// Beacon - holds field-centric x,y,angle coordinates
//         of targets on the field
Beacon::Beacon(void)
	: coord_(0,0,0)
{
}

Beacon::Beacon(const BeaconCoord &state)
	: coord_(state)
{
}

const BeaconCoord &Beacon::Get(void) const
{
	return coord_;
}

// From https://gist.github.com/volkansalma/2972237
static float atan2_approximation1(float y, float x)
{
    //http://pubs.opengroup.org/onlinepubs/009695399/functions/atan2.html
    //Volkan SALMA

    constexpr float ONEQTR_PI = M_PI / 4.0;
	constexpr float THRQTR_PI = 3.0 * M_PI / 4.0;
	const float abs_y = fabs(y) + 1e-10f;      // kludge to prevent 0/0 condition
	float r, angle;
	if ( x < 0.0f )
	{
		r = (x + abs_y) / (abs_y - x);
		angle = THRQTR_PI;
	}
	else
	{
		r = (x - abs_y) / (x + abs_y);
		angle = ONEQTR_PI;
	}
	angle += (0.1963f * r * r - 0.9817f) * r;
	if ( y < 0.0f )
		return( -angle );     // negate if in quad III or IV
	else
		return( angle );
}

// Calculate the distance and bearing from the beacon to the
// robot coord
// Set reverse_angle to true to measure the angle from
// the robot to the beacon instead
DistanceAndBearing Beacon::distance(const ParticleState &coord, bool reverse_angle) const
{
	float dx = coord_.x() - coord[0];
	float dy = coord_.y() - coord[1];
	if (reverse_angle)
	{
		dx = -dx;
		dy = -dy;
	}
	// hypotf is fairly slow
	// A few options if it needs to be sped up
	//  - find a faster implementation of sqrt
	//  - use the distance squared for everything
	//    (this will require detections to have their distance squared as well)
	const float distance = hypotf(dx, dy);
	// atan2 turns out to be a potential bottleneck in processing 
	// detection.getCosts().  See the link for possible optimizations
	// http://www-labs.iro.umontreal.ca/~mignotte/IFT2425/Documents/EfficientApproximationArctgFunction.pdf
	// for additional possible optimizations
	const float bearing  = atan2_approximation1(dy, dx);
	return DistanceAndBearing(distance, bearing);
}

double Beacon::robotBearing(const double angle) const
{
	const double abs_bearing = M_PI - angle;
	const double rel_bearing = abs_bearing - coord_.angle();
	return fast_normalize_angle(rel_bearing);
}

Beacons::Beacons(void)
{
}

Beacons::Beacons(const std::vector<Beacon> &beacons)
			: beacons_(beacons)
{
}

const Beacon& Beacons::operator[](size_t i) const
{
	if (i >= beacons_.size())
		throw std::out_of_range("Beacon index out of range");

	return beacons_[i];
}

void Beacons::append(const Beacon &beacon)
{
	beacons_.push_back(beacon);
}

void Beacons::append(const double x, const double y, const double angle)
{
	append(BeaconCoord(x, y, angle));
}

void Beacons::mirror_x(void)
{
	std::vector<Beacon> mirrored_beacons;
	for (auto &b : beacons_)
	{
		const auto coords = b.Get();
		mirrored_beacons.push_back(
				Beacon(BeaconCoord(
						- coords.x(),
					     coords.y(),
					     M_PI - coords.angle()
					)));
	}
	beacons_.insert(beacons_.end(), mirrored_beacons.begin(), mirrored_beacons.end());
}

void Beacons::mirror_y(void)
{
	std::vector<Beacon> mirrored_beacons;
	for (auto &b : beacons_)
	{
		const auto coords = b.Get();
		mirrored_beacons.push_back(
				Beacon(BeaconCoord(
						coords.x(),
					    - coords.y(),
					    2. * M_PI - coords.angle()
					)));
	}
	beacons_.insert(beacons_.end(), mirrored_beacons.begin(), mirrored_beacons.end());
}

size_t Beacons::size(void) const
{
	return beacons_.size();
}

