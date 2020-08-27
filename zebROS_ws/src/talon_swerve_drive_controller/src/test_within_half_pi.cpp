#include <random>
#include <iostream>
#include <ctime>
#include "talon_swerve_drive_controller/900Math.h"
#include <iomanip>


int main()
{
    //Type of random number distribution
    std::uniform_real_distribution<double> dist(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max());  //(min, max)

    //Mersenne Twister: Good quality random number generator
    std::mt19937 rng;
    //Initialize with non-deterministic seeds
    rng.seed(std::random_device{}());

    // generate 10 random numbers.
	size_t counter = 0;
	for (double currPos = -4.1 * M_PI; currPos <= 4.1 * M_PI; currPos += std::numeric_limits<double>::epsilon() * 1e10)
	{
		for (double targetPos = -2 * M_PI; targetPos <= 2 * M_PI; targetPos += std::numeric_limits<double>::epsilon() * 1e10)
		{
			if (counter == 0)
			{
				std::cout << std::setprecision(15) << currPos <<  " " << targetPos <<  std::endl;
				counter = 100000001;
			}
			counter -= 1;
			bool reverse;
			//	double currPos = dist(rng);
			//		double targetPos = dist(rng);

			const double ang = leastDistantAngleWithinHalfPi(currPos, targetPos, reverse);
			if (fabs(currPos - ang) >= (M_PI/2.0))
				std::cout << currPos <<  " " << targetPos <<  " " << ang << " " << static_cast<int>(reverse) << std::endl;
		}
	}
    return 0;
}
