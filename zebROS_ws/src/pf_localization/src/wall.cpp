#include "pf_localization/wall.hpp"

// Each wall is a line segment
Wall::Wall(double x0, double y0, double x1, double y1)
			: pStart_{x0, y0}
			, pEnd_{x1, y1}
{
}

Wall::Wall(const Eigen::Vector2d &pStart, const Eigen::Vector2d &pEnd)
	: pStart_(pStart)
	, pEnd_(pEnd)
{
}

double Wall::startX(void) const
{
	return pStart_[0];
}
double Wall::startY(void) const
{
	return pStart_[1];
}
double Wall::endX(void) const
{
	return pEnd_[0];
}
double Wall::endY(void) const
{
	return pEnd_[1];
}

const Eigen::Vector2d Wall::start(void) const
{
	return pStart_;
}

const Eigen::Vector2d Wall::end(void) const
{
	return pEnd_;
}

bool Wall::intersect(const Wall &wall) const
{
	return (ccw(pStart_, wall.start(), wall.end()) != ccw(pEnd_, wall.start(), wall.end())) &&
		   (ccw(pStart_, pEnd_, wall.start()) != ccw(pStart_, pEnd_, wall.end()));
}

// Returns true if line segments AB and CD intersect
bool Wall::ccw(const Eigen::Vector2d &A, const Eigen::Vector2d &B, const Eigen::Vector2d &C) const
{
	return (C[1] - A[1]) * (B[0] - A[0]) > ((B[1] - A[1]) * (C[0] - A[0]));
}


// Collection of all walls which make up the field.
// At this point, mostly just used for checking that predicted location
// falls within the field perimeter
Walls::Walls(void)
	: maxx_(-std::numeric_limits<double>::max())
	, maxy_(-std::numeric_limits<double>::max())
	, minx_(std::numeric_limits<double>::max())
    , miny_(std::numeric_limits<double>::max())
{
}

Walls::Walls(const std::vector<Wall> &walls)
	: walls_(walls)
	, maxx_(-std::numeric_limits<double>::max())
	, maxy_(-std::numeric_limits<double>::max())
	, minx_(std::numeric_limits<double>::max())
    , miny_(std::numeric_limits<double>::max())
{

	for (const auto wall : walls)
	{
		maxx_ = std::max(maxx_, std::max(wall.startX(), wall.endX()));
		maxy_ = std::max(maxy_, std::max(wall.startY(), wall.endY()));
		minx_ = std::min(minx_, std::min(wall.startX(), wall.endX()));
		miny_ = std::min(miny_, std::min(wall.startY(), wall.endY()));
	}
}

void Walls::append(const Wall &wall)
{
	walls_.push_back(wall);
	maxx_ = std::max(maxx_, std::max(wall.startX(), wall.endX()));
	maxy_ = std::max(maxy_, std::max(wall.startY(), wall.endY()));
	minx_ = std::min(minx_, std::min(wall.startX(), wall.endX()));
	miny_ = std::min(miny_, std::min(wall.startY(), wall.endY()));
}

// Check if a line segment (passed in as a wall)
// intersects with any wall segments in the Walls object
bool Walls::intersect(const Wall &wall) const
{
	for (const auto w: walls_)
	{
		if (w.intersect(wall))
			return true;
	}
	return false;
}

// Return true if the point in question falls
// outside the field boundaries
bool Walls::outsideField(const ParticleState &point) const
{
	if (point[0] > maxx_)
		return true;
	if (point[0] < minx_)
		return true;
	if (point[1] > maxy_)
		return true;
	if (point[1] < miny_)
		return true;

	return false;
}

