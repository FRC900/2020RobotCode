#include <iostream>
#include <eigen3/Core.h>

int main()
{
	Eigen::MatrixXd x;
	x = Eigen::MatrixXd::Random(10, 10);
	std::cout << x << std::endl;
	return 0;
}
