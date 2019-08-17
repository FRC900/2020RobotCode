#pragma once

// Particle filter class.
#include "pf_localization/beacon.hpp"
#include "pf_localization/detection.hpp"
#include "pf_localization/field_map.hpp"
#include "pf_localization/particle.hpp"

class ParticleFilter
{
	public:
		// B matrix is the input matrix - multiplied by inputs to get part of the next
		// timestep's robot state
		typedef Eigen::Matrix<double, STATE_STATES, STATE_INPUTS> BMatrix;

		// F is the system matrix - multiplied by the current robot state to
		// get the other part of the next timestep's robot state
		typedef Eigen::Matrix<double, STATE_STATES, STATE_STATES> FMatrix;
		ParticleFilter(double NP, double min_x, double max_x, double min_y, double max_y,
				const FieldMap &fieldMap, const FMatrix &F, const BMatrix &B,
				const Eigen::Matrix2d &Q, const Eigen::Matrix3d &R);

		// B matrix holds DT for the timestep to move the robot
		// the correct amount for the commanded velocity.
		// Since the timestep isn't guaranteed to be constant, allow
		// it to be set here.
		// Might make more sense to just make dt a parameter to moveParticles?
		void setB(const BMatrix &B);

		// Move each particle given the commanded input
		// vector (x', y', theta')
		// TODO - special case for robot not commanded to move at all?
		//        There, we won't be randomly drifting most of the time, so
		//        maybe don't add noise in that case and just make sure the state
		//        matrix ends up with the correct velocity.
		//        Then again, there is a little bit of drift after setting
		//        0 velocity, and we can also be rammed and moved so who knows?
		void moveParticles(const Eigen::Vector3d &u);

		// Since we have a trusted angle measurement, simply
		// set all particles to the desired angle plus
		// some noise
		void rotateParticles(double angle);

		// The estimated state is just the weighted sum
		// of all the particles
		void getEstimate(ParticleState &estimate) const;

		// Main localization function - given the dietections passed in
		// update weights and reample to get to new state
		void localize(Detections &detections);

	private:
	    // Given the current state plus commanded velocity, compute the
		// next state of a particle
	    // x is the current robot state.
		// u is the commanded velocity in x, y, theta
		// F_ and B_ control how the state plus velocity inputs
		// lead to the next state
		ParticleState motionModel(const ParticleState &x, const Eigen::Vector3d &u) const;

		// update the current set of particles to a new one
		// Do so by picking from the current set of particles
		// The chance of each particle being picked is its weight
		void resample(void);

		double NP_; // num particles;
		double NTh_; // threshold for resampling

		FieldMap fieldMap_; // set of beacons and walls which make up the field map
		BMatrix B_;         // state-space input matrix
		FMatrix F_;         // state-space system matrix
		Eigen::Matrix2d Q_; // Sensor covariance
		Eigen::Matrix3d R_; // Robot state covariance

		std::vector<ParticleState> particleStates_;
		std::vector<double>        particleWeights_;

		// Random number generator used in various parts of the class
		std::mt19937               randomGen_;
};

