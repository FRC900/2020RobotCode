#pragma once
#include <vector>
#include <Eigen/Dense>
#include "pf_localization/beacon.hpp"
#include "pf_localization/distance_and_bearing.hpp"
#include "pf_localization/field_map.hpp"
#include "pf_localization/particle.hpp"

#include "hungarian.hpp"
#include "branch_and_bound.hpp"

typedef unsigned int DetectionCostType;

// indicates no way that this beacon <-> detection pairing is physically possible
constexpr DetectionCostType invalidCost = std::numeric_limits<DetectionCostType>::max();
constexpr double maxDetectionDistance = 7.0; // meters

// Class to hold detection information. A detection is a sensor
// reading of a particular target. The detection holds robot-centric
// bearing and distance from the sensor measurement of that target.
// In addition, it can store the beacon that is the most likely match
// for that particular detection given the predicted location of the robot
//
// TODO - generalize this class.  DistanceAndBearing should be able to
// be a generic metric instead of a specific one.  That way we could swap in
// bearing-only detection from a non-depth RGB camera with minimal disruption
class Detection
{
	public:
		Detection(const DistanceAndBearing &coord);
		Detection(const double distance, const double bearing);

		void setActual(const Beacon &beacon);
		void resetActual(void);

		bool getActual(Beacon &actualBeacon) const;

		const DistanceAndBearing& getDistanceAndBearing(void) const;

		double weight(const ParticleState &loc, const Eigen::Matrix2d &Q) const;
		std::vector<DetectionCostType> getCosts(const ParticleState &robotLoc, const Beacons &beacons) const;

		friend std::ostream& operator<<(std::ostream &stream, const Detection &detection);

	private:
		DistanceAndBearing coord_;
		Beacon             actualBeacon_;
		bool               beaconValid_;

};

// Class to hold all detections seen in this timestep
class Detections
{
	public:
		Detections(void);

		void append(const Detection &detection);
		void append(const double distance, const double bearing);

		std::vector<Beacon> getActuals(void) const;

		double weights(const ParticleState &loc, const FieldMap &fieldMap, const Eigen::Matrix2d &Q);

		void guessActuals(const ParticleState &loc, const Beacons &beacons);

		friend std::ostream& operator<<(std::ostream &stream, const Detections &detections);

	private:
		std::vector<Detection> detections_;
//		AssignmentProblemSolver<DetectionCostType> solver_;
		BandBSolver<DetectionCostType> solver_;

};

