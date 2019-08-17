#pragma once
// from https://raw.githubusercontent.com/Smorodov/Multitarget-tracker/master/HungarianAlg/HungarianAlg.h
#include <vector>
#include <iostream>
#include <limits>
// http://community.topcoder.com/tc?module=Static&d1=tutorials&d2=hungarianAlgorithm
template <class T>
class AssignmentProblemSolver
{
	private:
		// --------------------------------------------------------------------------
		// Computes the optimal assignment (minimum overall costs) using Munkres algorithm.
		// --------------------------------------------------------------------------
		void assignmentoptimal(int *assignment, T *cost, T *distMatrix, size_t nOfRows, size_t nOfColumns);
		void buildassignmentvector(int *assignment, size_t nOfRows, size_t nOfColumns);
		void computeassignmentcost(int *assignment, T *cost, T *distMatrix, size_t nOfRows);
		void step2a(int *assignment, size_t nOfRows, size_t nOfColumns, size_t minDim);
		void step2b(int *assignment, size_t nOfRows, size_t nOfColumns, size_t minDim);
		void step3 (int *assignment, size_t nOfRows, size_t nOfColumns, size_t minDim);
		void step4 (int *assignment, size_t nOfRows, size_t nOfColumns, size_t minDim, size_t row, size_t col);
		void step5 (int *assignment, size_t nOfRows, size_t nOfColumns, size_t minDim);
		// --------------------------------------------------------------------------
		// Computes a suboptimal solution. Good for cases with many forbidden assignments.
		// --------------------------------------------------------------------------
		void assignmentsuboptimal1(int *assignment, T *cost, T *distMatrixIn, size_t nOfRows, size_t nOfColumns);
		// --------------------------------------------------------------------------
		// Computes a suboptimal solution. Good for cases with many forbidden assignments.
		// --------------------------------------------------------------------------
		void assignmentsuboptimal2(int *assignment, T *cost, T *distMatrixIn, size_t nOfRows, size_t nOfColumns);

		T     *distMatrix;
		size_t distMatrixElements;
		bool  *coveredColumns;
		size_t coveredColumnsElements;
		bool  *coveredRows;
		size_t coveredRowsElements;
		bool  *starMatrix;
		size_t starMatrixElements;
		bool  *newStarMatrix;
		size_t newStarMatrixElements;
		bool  *primeMatrix;
		size_t primeMatrixElements;

		void initBoolArray(bool *&array, size_t &size, size_t desiredSize);


	public:
		enum TMethod { optimal, many_forbidden_assignments, without_forbidden_assignments };
		AssignmentProblemSolver();
		~AssignmentProblemSolver();
		T Solve(std::vector<std::vector<T> >& DistMatrix, std::vector<int>& Assignment,TMethod Method=optimal);
};
