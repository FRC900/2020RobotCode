#include "pf_localization/detection.hpp"
#include "pf_localization/util.h"

Detection::Detection(const DistanceAndBearing &coord)
			: coord_(coord)
			, beaconValid_(false)
{
}

Detection::Detection(const double distance, const double bearing)
	: coord_(DistanceAndBearing(distance, bearing))
    , beaconValid_(false)
{
}

// Set, clear, get the actual beacon most likely
// associated with this detection.
void Detection::setActual(const Beacon &beacon)
{
	actualBeacon_ = beacon;
	beaconValid_  = true;
}

void Detection::resetActual(void)
{
	beaconValid_ = false;
}

bool Detection::getActual(Beacon &actualBeacon) const
{
	if (!beaconValid_)
		return false;
	actualBeacon = actualBeacon_;
	return true;
}

const DistanceAndBearing& Detection::getDistanceAndBearing(void) const
{
	return coord_;
}

static double gaussLikelihood(double x, double sigmaSq)
{
	return  exp(-(x * x) / (2.0 * sigmaSq)) / sqrt(2.0 * M_PI * sigmaSq);
}

double Detection::weight(const ParticleState &loc, const Eigen::Matrix2d &Q) const
{
	// TODO - maybe make this a small number rather than
	// zero to assign a penalty to particles which produce
	// cases where a detection can't be mapped to a beacon
	if (beaconValid_ == false)
		return 0;

	// distance / bearing to best predicted ideal detection
	auto dAndB = actualBeacon_.distance(loc);

	// difference in distance and bearing between predicted
	// and actual location - how far off the estimate is
	dAndB -= coord_;

	// given the variance Q, what is the likelihood that
	// this measurement corresponds to reality
	double w  = gaussLikelihood(dAndB.distance(), Q(0,0));
		   w *= gaussLikelihood(dAndB.bearing(),  Q(1,1));

	return w;
}

// For a set of beacons and the given robot location, find
// the cost from this detection to each of those beacons
// The cost is basically an error term between ideal and
// actual location - higher costs mean that the beacon is
// further from where it is expected to be.
// The set of all detections->beacons cost is used to assign
// each detection to a predicted beacon while minimizingt the
// total cost of the assignment.
std::vector<DetectionCostType> Detection::getCosts(const ParticleState &robotLoc, const Beacons &beacons) const
{
	std::vector<DetectionCostType> costs(beacons.size());

	for (size_t i = 0; i < beacons.size(); i++)
	{
		const auto robotDistanceAndBearing = beacons[i].distance(robotLoc);
		// No way we can detect this far out, so cost is set to infeasable
		if (robotDistanceAndBearing.distance() > maxDetectionDistance)
		{
			costs[i] = invalidCost;
			continue;
		}
		// No way we can detect beyond the limits of the FoV of the camera
		if (fabs(fast_normalize_angle(robotDistanceAndBearing.bearing() - robotLoc[2])) > 0.85)
		{
			costs[i] = invalidCost;
			continue;
		}
		// Check that the target-relative angle from target to robot is
		// between +/- pi/2. This is a quick way to rule
		// out cases where it would be looking through
		// a wall to see the target
		const double reverseBearing = fabs(beacons[i].robotBearing(coord_.bearing()));
		if (reverseBearing >= (M_PI / 2.0))
		{
			costs[i] = invalidCost;
			continue;
		}

		const double deltaDistance = fabs(coord_.distance() - robotDistanceAndBearing.distance());
		const double deltaBearing = fabs(coord_.bearing() - robotDistanceAndBearing.bearing());

		// Weight bearing higher than distance due to difference in
		// magnitude of m vs radians
		// TODO : tune this
		costs[i] = deltaDistance * 100. + deltaBearing * 5. * 100.;
	}
	return costs;
}

std::ostream& operator<<(std::ostream &stream, const Detection &detection)
{
	stream << "Detection:" << std::endl;
	stream << "\tcoord=" << detection.coord_;
	stream << "\tbeaconValid=" << detection.beaconValid_;
	stream << "\tactualBeacon=" << detection.actualBeacon_;
	return stream;
}

Detections::Detections(void)
{
}

// Add a detection to the list
void Detections::append(const Detection &detection)
{
	detections_.push_back(detection);
}

void Detections::append(const double distance, const double bearing)
{
	detections_.push_back(Detection(distance, bearing));
}

// Get a list of Beacons mapped to each Detection
std::vector<Beacon> Detections::getActuals(void) const
{
	std::vector<Beacon> actuals;
	for (const auto d: detections_)
	{
		Beacon b;
		if (d.getActual(b))
			actuals.push_back(b);
	}
	return actuals;
}

// Calculate the combined weight for all of the detections
// Basically, the weight is the probability that the each of the
// detections is close enough to ideal to be accurate.
// So higher weights are better
// The process -
//   call guessActuals to correlate each detection with a beacon
//   multiply the weight calculated from each of those assignments
double Detections::weights(const ParticleState &loc, const FieldMap &fieldMap, const Eigen::Matrix2d &Q)
{
	// Particles outside the field are impossible
	// TODO Maybe add a small weight for those close to the boundary?
	if (fieldMap.outsideField(loc))
		return 0;

	guessActuals(loc, fieldMap.getBeacons());
	double w = 1;
	for (const auto d : detections_)
		w *= d.weight(loc, Q);
	return w;
}

// Use hungarian algorithm to find a min-cost assignment
// between detections and beacon locations on the map.
// Since beacons are all the same, the code doesn't know which
// beacon each detection corresponds to.  Generate a cost matrix -
// basically the error in distance and bearing between each detection
// and where the beacon should be detected if loc were accurate
// Find a mapping from each detection to a beacon such that the
// overall distance is minimized
void Detections::guessActuals(const ParticleState &loc, const Beacons &beacons)
{
	std::vector<std::vector<DetectionCostType>> costs(detections_.size());
	//std::cout << "costs" << std::endl;
	for (size_t i = 0; i < detections_.size(); i++)
	{
		costs[i] = detections_[i].getCosts(loc, beacons);
#if 0
		for (size_t j = 0; j < costs[i].size(); j++)
		{
			if (j)
				std::cout << ",";
			std::cout << costs[i][j];
		}
		std::cout << std::endl;
#endif
	}
	// TODO - if an entire column or row is infeasable, remove it
	// from the assignment run. Need to keep a map correlating beacons
	// to which column represents each (it won't be a simple 1:1 mapping anymore)
	// If a row is removed, it means the detection can't be matched to
	// any beacons - need to remember this later and correctly reset
	// the detection.
	// A fix for this might be to reset all of the actuals unconditionally
	// and then set the assignments for the entries which do exist.  This
	// will skip over the ones which are removed, and will leave them
	// unset.
	std::vector<int> assignment;
	solver_.Solve(costs, assignment);

	//std::cout << "assignment" << std::endl;
	for (size_t i = 0; i < assignment.size(); ++i)
	{
#if 0
		if (i)
			std::cout << ",";
		std::cout << assignment[i];
#endif
		if (assignment[i] == -1 || (costs[i][assignment[i]] > 1e4)) // TODO - no magic numbers
			detections_[i].resetActual();
		else
			detections_[i].setActual(beacons[assignment[i]]);
	}
	//std::cout << std::endl;
}

std::ostream& operator<<(std::ostream &stream, const Detections &detections)
{
	stream << "Detections:" << std::endl;
	for (size_t i = 0; i < detections.detections_.size(); i++)
		stream << "\t[" << i << "]=" << detections.detections_[i] << std::endl;
	return stream;
}

