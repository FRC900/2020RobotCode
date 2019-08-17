#include <chrono>
#include <random>

#include <Eigen/Dense>

#include <pf_localization/particle_filter.hpp>


ParticleFilter::ParticleFilter(double NP, double min_x, double max_x, double min_y, double max_y,
			const FieldMap &fieldMap, const FMatrix &F, const BMatrix &B,
			const Eigen::Matrix2d &Q, const Eigen::Matrix3d &R)
	: NP_(NP) // num particles
	, NTh_(NP/2.0) // threshold for resampling
	, fieldMap_(fieldMap) // map of the field - includes both beacons and walls
	, B_(B)
	, F_(F)
	, Q_(Q) // sensor covariance matrix
	, R_(R) // robot state covariance matrix
	  , randomGen_(std::chrono::system_clock::now().time_since_epoch().count())
{
	// Create initial particle state - randomly distributed over the
	// entire rectangle ((min_x,min_y), (max_x, max_y))
	// Theta will be updated almost immediately from IMU data -
	// or maybe a TODO is to init this with an IMU heading?
	std::uniform_real_distribution<double> x_distribution(min_x, max_x);
	std::uniform_real_distribution<double> y_distribution(min_y, max_y);

	for (size_t i = 0; i < NP_; ++i)
	{
		ParticleState ps;
		ps << x_distribution(randomGen_), y_distribution(randomGen_), 0,0,0,0;
		particleStates_.push_back(ps);
		particleWeights_.push_back(1./NP);
	}
}

// B matrix holds DT for the timestep to move the robot
// the correct amount for the commanded velocity.
// Since the timestep isn't guaranteed to be constant, allow
// it to be set here.
// Might make more sense to just make dt a parameter to moveParticles?
void ParticleFilter::setB(const BMatrix &B)
{
	B_ = B;
}

// Move each particle given the commanded input
// vector (x', y', theta')
// TODO - special case for robot not commanded to move at all?
//        There, we won't be randomly drifting most of the time, so
//        maybe don't add noise in that case and just make sure the state
//        matrix ends up with the correct velocity.
//        Then again, there is a little bit of drift after setting
//        0 velocity, and we can also be rammed and moved so who knows?
void ParticleFilter::moveParticles(const Eigen::Vector3d &u)
{
	std::uniform_real_distribution<double> randn(0.0, 1.0);
	for (auto &ps : particleStates_)
	{
		// Add a small amount of random noise to the
		// velocity given to each particle
		Eigen::Vector3d ud
		(
			u(0) + randn(randomGen_) * R_(0,0),
			u(1) + randn(randomGen_) * R_(1,1),
			u(2) + randn(randomGen_) * R_(2,2)
		);
		ps = motionModel(ps, ud);
	}
}

// Since we have a trusted angle measurement, simply
// set all particles to the desired angle plus
// some noise
void ParticleFilter::rotateParticles(double angle)
{
	std::uniform_real_distribution<double> randn(0.0, 1.0);
	for (auto &ps : particleStates_)
		ps(2) = angle + randn(randomGen_) * R_(2,2);
}

// The estimated state is just the weighted sum
// of all the particles
void ParticleFilter::getEstimate(ParticleState &estimate) const
{
	estimate = ParticleState::Zero();
	for (size_t i = 0; i < NP_; ++i)
		estimate += particleStates_[i] * particleWeights_[i];
}

// Main localization function - given the dietections passed in
// update weights and reample to get to new state
void ParticleFilter::localize(Detections &detections)
{
	double sumOfWeights = 0.0;
	for (size_t i = 0; i < NP_; ++i)
	{
		particleWeights_[i] = detections.weights(particleStates_[i], fieldMap_, Q_);
		sumOfWeights += particleWeights_[i];
	}

	// Normalize weights so they sum to 1
	if (sumOfWeights != 0)
		for (auto &w: particleWeights_)
			w /= sumOfWeights;

	resample();
}

// Given the current state plus commanded velocity, compute the
// next state of a particle
// x is the current robot state.
// u is the commanded velocity in x, y, theta
// F_ and B_ control how the state plus velocity inputs
// lead to the next state
ParticleState ParticleFilter::motionModel(const ParticleState &x, const Eigen::Vector3d &u) const
{
	return F_ * x + B_ * u;
}
// update the current set of particles to a new one
// Do so by picking from the current set of particles
// The chance of each particle being picked is its weight
void ParticleFilter::resample(void)
{
	// TODO - check Nth to see if resampling is needed
	std::discrete_distribution<size_t> index(particleWeights_.begin(), particleWeights_.end());
	std::vector<ParticleState> resampledStates;
	for (size_t i = 0; i < NP_; ++i)
	{
				// j is the index of the particle selected from the old state
		size_t j = index(randomGen_);
		// TODO - add to me.  
		// Look at code in https://github.com/mithi/particle-filter-prototype/blob/master/kidnapped-vehicle/src/particle_filter.cpp, resample() for an example
		// Add that particle from the set of old particles to the
		// list of the new ones
		// Reset the weight of the new particle to 1/NP_
	}
	// copy the newly created particle array - resamplesStates
	// to particleWeights_;
}
