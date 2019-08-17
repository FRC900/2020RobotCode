#pragma once

#include <vector>
#include <Eigen/Dense>

#include <pf_localization/particle.hpp>

// Class holding the start and end coordinates of a wall segment
class Wall
{
	public:
		Wall(double x0, double y0, double x1, double y1);
		Wall(const Eigen::Vector2d &pStart, const Eigen::Vector2d &pEnd);

		double startX(void) const;
		double startY(void) const;
		double endX(void) const;
		double endY(void) const;

		const Eigen::Vector2d start(void) const;

		const Eigen::Vector2d end(void) const;

		bool intersect(const Wall &wall) const;

	private:
		// Returns true if line segments AB and CD intersect
		bool ccw(const Eigen::Vector2d &A, const Eigen::Vector2d &B, const Eigen::Vector2d &C) const;
		Eigen::Vector2d pStart_;
		Eigen::Vector2d pEnd_;
};


// A collection of walls which make up a complete field map
class Walls
{
	public:
		Walls(void);
		Walls(const std::vector<Wall> &walls);

		void append(const Wall &wall);

		bool intersect(const Wall &wall) const;

		bool outsideField(const ParticleState &point) const;
	private:
		std::vector<Wall> walls_;
		double maxx_;
		double maxy_;
		double minx_;
		double miny_;
};


