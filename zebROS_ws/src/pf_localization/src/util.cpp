#include <cmath>

// The angles we'll generate are the result of atan2 plus / minus
// in the range around -2Pi to 2Pi.  This should be quick in that
// case - faster than the generally-better fmod solution?
double fast_normalize_angle(double angle)
{
	if (angle > M_PI)
	{
		do
		{
			angle -= 2.0 * M_PI;
		}
		while(angle > M_PI);
	}
	else if (angle <= -M_PI)
	{
		do
		{
			angle += 2.0 * M_PI;
		}
		while(angle <= -M_PI);
	}
	return angle;
}
