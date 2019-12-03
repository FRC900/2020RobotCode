#pragma once
// Initially from https://www.geeksforgeeks.org/job-assignment-problem-using-branch-and-bound/

// Program to solve Job Assignment problem
// using Branch and Bound
#include <iostream>
#include <limits>
#include <memory>
#include <vector>
#include <queue>

// state space tree node
template <class CostType>
class Node
{
public:

	Node(const size_t jobCount)
		: workerID_(-1)
		, jobID_(-1)
		, assigned_(jobCount, false)
		, parent_(nullptr)
	{
	}

	// Function to allocate a new search tree node
	// Here Person workerID is assigned to job jobID
	Node(size_t jobCount, int workerID, int jobID, std::shared_ptr<Node> parent)
		: workerID_(workerID)
		, jobID_(jobID)
		, assigned_(jobCount)
		, parent_(parent)
	{
		for (size_t i = 0; i < jobCount; i++)
		{
			if (!parent_)
				assigned_[i] = false;
			else
				assigned_[i] = parent->isAssigned(i);
		}

		if (jobID >= 0)
			assigned_[jobID] = true;
	}

	CostType getCost(void) const
	{
		return cost_;
	}

	void setCost(const CostType cost)
	{
		cost_ = cost;
	}

	CostType getPathCost(void) const
	{
		return pathCost_;
	}

	void setPathCost(const CostType pathCost)
	{
		pathCost_ = pathCost;
	}

	void printAssignments(void) const
	{
		if (parent_ == NULL)
			return;

		parent_->printAssignments();
		std::cout << "Assign Worker " << char(workerID_ + 'A')
			<< " to Job " << jobID_ << std::endl;
	}
	void getAssignments(std::vector<int> &assignments) const
	{
		if (parent_ == nullptr)
		{
			assignments.clear();
		}
		else
		{
			parent_->getAssignments(assignments);
			assignments.push_back(jobID_);
		}
	}

	int getWorkerID(void) const
	{
		return workerID_;
	}

	bool isAssigned(const size_t jobID) const
	{
		return assigned_[jobID];
	}

	void print(size_t jobCount) const
	{
		std::cout << "Node Worker " << workerID_ << " gets Job " << jobID_ <<
			" pathCost = " << pathCost_ << " cost = " << cost_ << std::endl;
		for (size_t i = 0; i < jobCount; i++)
			std::cout << "  assigned[" << i << "] = " << assigned_[i] << std::endl;
	}
	void printTree(size_t jobCount) const
	{
		if (parent_ == nullptr)
			return;
		parent_->printTree(jobCount);
		print(jobCount);
	}

	// Function to calculate the least promising cost
	// of node after worker x is assigned to job y.
	CostType calculateCost(const std::vector<std::vector<CostType>> &costMatrix,
			std::vector<bool> &available) const
	{
		CostType cost = 0;

		// to store unavailable jobs
		// Start with current set of unassigned jobs, and updated
		// them as they are given to workers
		//auto available = std::make_unique<bool []>(costMatrix[0].size());
		//std::vector<bool> available(costMatrix[0].size());
		for (size_t i = 0; i < costMatrix[0].size(); i++)
			available[i] = !assigned_[i];

		// start from next worker, assign each remaining lowest-cost job in
		// order to the next worker in the list
		for (size_t i = workerID_ + 1; i < costMatrix.size(); i++)
		{
			CostType min = std::numeric_limits<CostType>::max();
			int minIndex = -1;

			// do for each job
			for (size_t j = 0; j < costMatrix[i].size(); j++)
			{
				// if job is unassigned
				if (available[j] && (costMatrix[i][j] < min))
				{
					// store job number
					minIndex = j;

					// store cost
					min = costMatrix[i][j];
				}
			}

			if (minIndex != -1)
			{
				// add cost of next worker
				cost += min;

				// job becomes unavailable
				available[minIndex] = false;
			}
		}

		return cost;
	}


private:
	// contain worker number
	int workerID_;

	// contains Job ID
	int jobID_;

	// Boolean array assigned will contains
	// info about available jobs
	std::vector<bool> assigned_;

	// stores parent node of current node
	// helps in tracing path when answer is found
	std::shared_ptr<Node> parent_;

	// contains cost for ancestors nodes
	// including current node
	CostType pathCost_;

	// contains least promising cost
	CostType cost_;
};

// Comparison object to be used to order the heap

template <class CostType>
struct comp
{
	bool operator()(const std::shared_ptr<Node<CostType>> &lhs,
					const std::shared_ptr<Node<CostType>> &rhs) const
	{
		return lhs->getCost() > rhs->getCost();
	}
};

template <class CostType>
class BandBSolver
{
	typedef Node<CostType> NodeType;
	public:
		CostType Solve(std::vector<std::vector<CostType>> &costMatrix,
					   std::vector<int> &assignments)
		{
			size_t jobCount = 0;
			for (size_t i = 0; i < costMatrix.size(); i++)
				jobCount = std::max(jobCount, costMatrix[i].size());

			// Make matrix rectangular - this shouldn't happen, so
			// maybe flag as an error or warning instead?
			for (size_t i = 0; i < costMatrix.size(); i++)
				for (size_t j = costMatrix[i].size(); j < jobCount; j++)
					costMatrix[i].push_back(std::numeric_limits<CostType>::max());

			if (costMatrix.size() <= jobCount)
			{
				return findMinCostImpl(costMatrix, assignments);
			}
			// Transpose the matrix - this branch and bound algorithm
			// only works if cols >= rows.  If rows>cols, swap them
			// around, run the algorithm, then invert the returned
			// row<->col mapping to match the original, pre transpose
			// input
			std::vector<std::vector<CostType>> invCostMatrix(jobCount);
			for (size_t i = 0; i < jobCount; i++)
			{
				invCostMatrix[i] = std::vector<CostType>(costMatrix.size());
				for(size_t j = 0; j < costMatrix.size(); j++)
					invCostMatrix[i][j] = costMatrix[j][i];
			}
			std::vector<int> invAssignments;
			const double cost = findMinCostImpl(invCostMatrix, invAssignments);

			// Initialize to -1, fill in mappings that were found
			// The remaining ones will be left as -1, indicating
			// there is no job assigned to a particular worker
			assignments = std::vector<int>(costMatrix.size(), -1);
			for (size_t i = 0; i < invAssignments.size(); i++)
			{
				assignments[invAssignments[i]] = i;
			}
			return cost;
		}
	private:
		// Finds minimum cost using Branch and Bound.
		CostType findMinCostImpl(const std::vector<std::vector<CostType>> &costMatrix,
								 std::vector<int> &assignments)
		{
			const size_t workerCount = costMatrix.size();
			const size_t jobCount = costMatrix[0].size(); // TODO Check that all elements of costMatrix are the same size
			// Create a priority queue to store live nodes of
			// search tree;
			std::priority_queue<std::shared_ptr<NodeType>,
								std::vector<std::shared_ptr<NodeType>>,
								comp<CostType>> pq;

			std::vector<bool> availableBuffer(jobCount);

			// initailize heap to dummy node with cost 0
			// Add dummy node to list of live nodes;
			pq.push(std::make_shared<NodeType>(jobCount));

			// Finds a live node with least cost,
			// add its childrens to list of live nodes and
			// finally deletes it from the list.
			while (!pq.empty())
			{
				// Find a live node with least estimated cost
				auto min = pq.top();

				// The found node is deleted from the list of
				// live nodes
				pq.pop();

				// i stores next worker
				int i = min->getWorkerID() + 1;

				// if all workers are assigned a job
				if (i >= workerCount)
				{
					//min->printAssignments();
					min->getAssignments(assignments);
					return min->getCost();
				}

				// do for each job
				for (size_t j = 0; j < jobCount; j++)
				{
					// If unassigned and an assignment is not ruled out
					// by an infeasable cost
					if (!min->isAssigned(j) && (costMatrix[i][j] != std::numeric_limits<CostType>::max()))
					{
						// create a new tree node
						auto child = std::make_shared<NodeType>(jobCount, i, j, min);

						// cost for ancestors nodes including current node
						child->setPathCost(min->getPathCost() + costMatrix[i][j]);

						// calculate its lower bound
						child->setCost(child->getPathCost() + child->calculateCost(costMatrix, availableBuffer));

						// Add child to list of live nodes
						pq.push(child);
					}
				}
			}
			assignments.clear();
			return -1;
		}
};

#if 0
// Driver code
int main()
{
	typedef unsigned int CostType;
	// x-cordinate represents a Worker
	// y-cordinate represents a Job
	std::vector<std::vector<CostType>> costMatrix;
#if 0
	costMatrix.push_back(std::vector<CostType>{9, 2, 7, 8});
	costMatrix.push_back(std::vector<CostType>{3, 4, 6, 7});
	costMatrix.push_back(std::vector<CostType>{5, 8, 8, 1});
	costMatrix.push_back(std::vector<CostType>{6, 8, 7, 9});
	costMatrix.push_back(std::vector<CostType>{7, 6, 4, 9});
#endif


	costMatrix.push_back(std::vector<CostType>{
			504,42949295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,99,4294967295,4294967295,4294967295,4294967295,4294967295,42949672,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295});
	costMatrix.push_back(std::vector<CostType>{
			404,4294967295,4294967295,42949672,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,199,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295});
	costMatrix.push_back(std::vector<CostType>{
			394,414,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,610,778,4294967295,4294967295,4294967295,4294967295,4294967295,42949695,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295,4294967295});
	/* int costMatrix[N][N] =
	   {
	   {82, 83, 69, 92},
	   {77, 37, 49, 92},
	   {11, 69, 5, 86},
	   { 8, 9, 98, 23}
	   };
	   */

	/* int costMatrix[N][N] =
	   {
	   {2500, 4000, 3500},
	   {4000, 6000, 3500},
	   {2000, 4000, 2500}
	   };*/

	/*int costMatrix[N][N] =
	  {
	  {90, 75, 75, 80},
	  {30, 85, 55, 65},
	  {125, 95, 90, 105},
	  {45, 110, 95, 115}
	  };*/


std::vector<int> assignments;
	BandBSolver<CostType> solver;
	std::cout << "\nOptimal Cost is "
		 << solver.Solve(costMatrix, assignments)
		 << std::endl;
	for (size_t i = 0; i < assignments.size(); ++i)
	{
		if (i)
			std::cout << ",";
		std::cout << assignments[i];
	}
	std::cout << std::endl;

	return 0;
}

#endif
