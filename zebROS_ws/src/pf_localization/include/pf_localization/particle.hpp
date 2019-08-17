// File defining particle state - the proposed robot
// position and velocity for a given robot location hypothesis
#pragma once

constexpr size_t STATE_STATES = 6; // position & velocity in x, y, theta
constexpr size_t STATE_INPUTS = 3; // velocity in x, y, theta

// state of a given particle - position + velocity in x, y, theta
typedef Eigen::Matrix<double, 6, 1> ParticleState;

#if 0
// Each particle represents a guess at a state of the robot
// Here, we track both position and velocity in x, y and theta
struct ParticleState
{
	ParticleState()
	{
		m_ << 0, 0, 0, 0, 0, 0;
	}
	ParticleState(double xPos, double yPos)
	{
		m_ << xPos, yPos, 0, 0, 0, 0;
	}
	ParticleState(const Eigen::Matrix<double, 6, 1> &m)
		: m_(m)
	{
	}
	operator const Eigen::Matrix<double, 6, 1>()
	{
		return m_;
	}
	void setAngle(double angle)
	{
		m_(2) = angle;
	}
	private :
		Eigen::Matrix<double, 6, 1> m_;
};
#endif

