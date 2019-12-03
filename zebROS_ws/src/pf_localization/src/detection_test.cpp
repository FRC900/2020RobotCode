#include <iostream>
#include <chrono>

#include <pf_localization/detection.hpp>

double radians(double degrees) { return degrees * M_PI / 180.0; }

int main(void)
{
	Beacons beacons;

	beacons.append(2.07-.005,3.63-0.005, radians(270 - 60));
	beacons.append(2.44,3.39-.001, 3. * M_PI / 2.);
	beacons.append(2.81+0.0075,3.63-0.0075, radians(270 + 60));
	beacons.append(2.66,0.28, 0);
	beacons.append(1.64,0.71, M_PI / 2.);
	beacons.append(1.08,0.71, M_PI / 2.);
	beacons.append(0.53,0.71, M_PI / 2.);
	beacons.append(8.26 - 0.021,3.44, M_PI);
	beacons.mirror_x();
	beacons.mirror_y();
	std::cout << beacons << std::endl;

	Detections detections;

	detections.append(1.086, .14437);
	detections.append(2.086, .14437);
	detections.append(4.8  , -.6555);
	std::cout << detections << std::endl;


	ParticleState particle_state;

	particle_state << -3.372, 3.589, 2.078 - M_PI / 2., 0, 0, 0;

	const std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
	for (size_t i = 0; i < 1000000; i ++)
	{
		detections.guessActuals(particle_state, beacons);
	}
	std::cout << "After guessActuals : " << detections << std::endl;
	const std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
	const std::chrono::duration<double> elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
	std::cout << "Elapsed time : " << elapsed_time.count() << std::endl;

	return 0;
}
