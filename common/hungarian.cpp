// From https://raw.githubusercontent.com/Smorodov/Multitarget-tracker/master/HungarianAlg/HungarianAlg.cpp
#include <limits>
#include "hungarian.hpp"

using namespace std;

template <class T>
AssignmentProblemSolver<T>::AssignmentProblemSolver()
	: distMatrix(nullptr)
	, distMatrixElements(0)
	, coveredColumns(nullptr)
	, coveredColumnsElements(0)
	, coveredRows(nullptr)
	, coveredRowsElements(0)
	, starMatrix(nullptr)
	, starMatrixElements(0)
	, newStarMatrix(nullptr)
	, newStarMatrixElements(0)
	, primeMatrix(nullptr)
	, primeMatrixElements(0)
{
}

template <class T>
AssignmentProblemSolver<T>::~AssignmentProblemSolver()
{
	if (distMatrix)
		delete [] distMatrix;
	if (coveredColumns)
		delete [] coveredColumns;
	if (coveredRows)
		delete [] coveredRows;
	if (starMatrix)
		delete [] starMatrix;
	if (newStarMatrix)
		delete [] newStarMatrix;
	if (primeMatrix)
		delete [] primeMatrix;
}

template <class T>
T AssignmentProblemSolver<T>::Solve(vector<vector<T>>& DistMatrix,vector<int>& Assignment,TMethod Method)
{
	const size_t N=DistMatrix.size(); // number of columns (tracks)
	const size_t M=DistMatrix[0].size(); // number of rows (measurements)

	int *assignment	=new int[N];
	T *distIn		=new T[N*M];

	T  cost;
	// Fill matrix with random numbers
	for(size_t i=0; i<N; i++)
	{
		for(size_t j=0; j<M; j++)
		{
			distIn[i+N*j] = DistMatrix[i][j];
		}
	}
	switch(Method)
	{
		case optimal: assignmentoptimal(assignment, &cost, distIn, N, M); break;

		case many_forbidden_assignments: assignmentsuboptimal1(assignment, &cost, distIn, N, M); break;

		case without_forbidden_assignments: assignmentsuboptimal2(assignment, &cost, distIn, N, M); break;
	}

	// form result
	Assignment.clear();
	for(size_t x=0; x<N; x++)
	{
		Assignment.push_back(assignment[x]);
	}

	delete[] assignment;
	delete[] distIn;
	return cost;
}

template <class T>
void AssignmentProblemSolver<T>::initBoolArray(bool *&array, size_t &size, const size_t desiredSize)
{
	if (desiredSize > size)
	{
		if (array)
			delete [] array;
		array = new bool[desiredSize];
		size = desiredSize;
	}
	std::fill(array, array + desiredSize, false);
}
// --------------------------------------------------------------------------
// Computes the optimal assignment (minimum overall costs) using Munkres algorithm.
// --------------------------------------------------------------------------
template <class T>
void AssignmentProblemSolver<T>::assignmentoptimal(int *assignment, T *cost, T *distMatrixIn, size_t nOfRows, size_t nOfColumns)
{
	T *distMatrixTemp;
	T *distMatrixEnd;
	T  minValue;

	size_t nOfElements;
	size_t minDim;
	size_t row;

	// Init
	*cost = 0;
	for(row=0; row<nOfRows; row++)
	{
		assignment[row] = -1;
	}

	// Generate distance matrix
	// and check matrix elements positiveness :)

	// Total elements number
	nOfElements   = nOfRows * nOfColumns;
	// Memory allocation
	if (nOfElements > distMatrixElements)
	{
		if (distMatrix)
			delete [] distMatrix;
		distMatrix = new T[nOfElements];
		distMatrixElements = nOfElements;
	}
	// Pointer to last element
	distMatrixEnd = distMatrix + nOfElements;

	//
	for(row=0; row<nOfElements; row++)
	{
		T value = distMatrixIn[row];
		if(value < 0)
		{
			cout << "All matrix elements have to be non-negative." << endl;
		}
		distMatrix[row] = value;
	}

	// Memory allocation
	initBoolArray(coveredColumns, coveredColumnsElements, nOfColumns);
	initBoolArray(coveredRows,    coveredRowsElements,    nOfRows);
	initBoolArray(starMatrix,     starMatrixElements,     nOfElements);
	initBoolArray(primeMatrix,    primeMatrixElements,    nOfElements);
	initBoolArray(newStarMatrix,  newStarMatrixElements,  nOfElements);

	/* preliminary steps */
	if(nOfRows <= nOfColumns)
	{
		minDim = nOfRows;
		for(row=0; row<nOfRows; row++)
		{
			/* find the smallest element in the row */
			distMatrixTemp = distMatrix + row;
			minValue = *distMatrixTemp;
			distMatrixTemp += nOfRows;
			while(distMatrixTemp < distMatrixEnd)
			{
				T value = *distMatrixTemp;
				if(value < minValue)
				{
					minValue = value;
				}
				distMatrixTemp += nOfRows;
			}
			/* subtract the smallest element from each element of the row */
			distMatrixTemp = distMatrix + row;
			while(distMatrixTemp < distMatrixEnd)
			{
				*distMatrixTemp -= minValue;
				distMatrixTemp += nOfRows;
			}
		}
		/* Steps 1 and 2a */
		for(row=0; row<nOfRows; row++)
		{
			for(size_t col=0; col<nOfColumns; col++)
			{
				if ((distMatrix[row + nOfRows*col] == 0) && !coveredColumns[col])
				{
					starMatrix[row + nOfRows*col] = true;
					coveredColumns[col]           = true;
					break;
				}
			}
		}
	}
	else /* if(nOfRows > nOfColumns) */
	{
		minDim = nOfColumns;
		for(size_t col=0; col<nOfColumns; col++)
		{
			/* find the smallest element in the column */
			distMatrixTemp = distMatrix     + nOfRows*col;
			T *columnEnd      = distMatrixTemp + nOfRows;
			minValue = *distMatrixTemp++;
			while(distMatrixTemp < columnEnd)
			{
				T value = *distMatrixTemp++;
				if(value < minValue)
				{
					minValue = value;
				}
			}
			/* subtract the smallest element from each element of the column */
			distMatrixTemp = distMatrix + nOfRows*col;
			while(distMatrixTemp < columnEnd)
			{
				*distMatrixTemp++ -= minValue;
			}
		}
		/* Steps 1 and 2a */
		for(size_t col=0; col<nOfColumns; col++)
		{
			for(row=0; row<nOfRows; row++)
			{
				if((distMatrix[row + nOfRows*col] == 0) && !coveredRows[row])
				{
					starMatrix[row + nOfRows*col] = true;
					coveredColumns[col]           = true;
					coveredRows[row]              = true;
					break;
				}
			}
		}

		for(row=0; row<nOfRows; row++)
		{
			coveredRows[row] = false;
		}
	}
	/* move to step 2b */
	step2b(assignment, nOfRows, nOfColumns, minDim);
	/* compute cost and remove invalid assignments */
	computeassignmentcost(assignment, cost, distMatrixIn, nOfRows);
	return;
}
// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------
template <class T>
void AssignmentProblemSolver<T>::buildassignmentvector(int *assignment, size_t nOfRows, size_t nOfColumns)
{
	for(size_t row=0; row<nOfRows; row++)
	{
		for(size_t col=0; col<nOfColumns; col++)
		{
			if(starMatrix[row + nOfRows*col])
			{
				assignment[row] = col;
				break;
			}
		}
	}
}
// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------
template <class T>
void AssignmentProblemSolver<T>::computeassignmentcost(int *assignment, T *cost, T *distMatrix, size_t nOfRows)
{
	for(size_t row=0; row<nOfRows; row++)
	{
		int col = assignment[row];
		if(col >= 0)
		{
			*cost += distMatrix[row + nOfRows*col];
		}
	}
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------
template <class T>
void AssignmentProblemSolver<T>::step2a(int *assignment, size_t nOfRows, size_t nOfColumns, size_t minDim)
{
	/* cover every column containing a starred zero */
	for(size_t col=0; col<nOfColumns; col++)
	{
		bool *starMatrixTemp = starMatrix     + nOfRows*col;
		bool *columnEnd      = starMatrixTemp + nOfRows;
		while(starMatrixTemp < columnEnd)
		{
			if(*starMatrixTemp++)
			{
				coveredColumns[col] = true;
				break;
			}
		}
	}
	/* move to step 3 */
	step2b(assignment, nOfRows, nOfColumns, minDim);
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------
template <class T>
void AssignmentProblemSolver<T>::step2b(int *assignment, size_t nOfRows, size_t nOfColumns, size_t minDim)
{
	/* count covered columns */
	size_t nOfCoveredColumns = 0;
	for(size_t col=0; col<nOfColumns; col++)
	{
		if(coveredColumns[col])
		{
			nOfCoveredColumns++;
		}
	}
	if(nOfCoveredColumns == minDim)
	{
		/* algorithm finished */
		buildassignmentvector(assignment, nOfRows, nOfColumns);
	}
	else
	{
		/* move to step 3 */
		step3(assignment, nOfRows, nOfColumns, minDim);
	}
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------
template <class T>
void AssignmentProblemSolver<T>::step3(int *assignment, size_t nOfRows, size_t nOfColumns, size_t minDim)
{
	bool zerosFound = true;
	while(zerosFound)
	{
		zerosFound = false;
		for(size_t col=0; col<nOfColumns; col++)
		{
			if(!coveredColumns[col])
			{
				for(size_t row=0; row<nOfRows; row++)
				{
					if((!coveredRows[row]) && (distMatrix[row + nOfRows*col] == 0))
					{
						/* prime zero */
						primeMatrix[row + nOfRows*col] = true;
						/* find starred zero in current row */
						size_t starCol;
						for(starCol=0; starCol<nOfColumns; starCol++)
							if(starMatrix[row + nOfRows*starCol])
							{
								break;
							}
						if(starCol == nOfColumns) /* no starred zero found */
						{
							/* move to step 4 */
							step4(assignment, nOfRows, nOfColumns, minDim, row, col);
							return;
						}
						coveredRows[row]        = true;
						coveredColumns[starCol] = false;
						zerosFound              = true;
						break;
					}
				}
			}
		}
	}
	/* move to step 5 */
	step5(assignment, nOfRows, nOfColumns, minDim);
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------
template <class T>
void AssignmentProblemSolver<T>::step4(int *assignment, size_t nOfRows, size_t nOfColumns, size_t minDim, size_t row, size_t col)
{
	const size_t nOfElements = nOfRows*nOfColumns;
	/* generate temporary copy of starMatrix */
	for(size_t n=0; n<nOfElements; n++)
	{
		newStarMatrix[n] = starMatrix[n];
	}
	/* star current zero */
	newStarMatrix[row + nOfRows*col] = true;
	/* find starred zero in current column */
	size_t starCol = col;
	size_t starRow;
	for(starRow=0; starRow<nOfRows; starRow++)
	{
		if(starMatrix[starRow + nOfRows*starCol])
		{
			break;
		}
	}
	while(starRow<nOfRows)
	{
		/* unstar the starred zero */
		newStarMatrix[starRow + nOfRows*starCol] = false;
		/* find primed zero in current row */
		size_t primeRow = starRow;
		size_t primeCol;
		for(primeCol=0; primeCol<nOfColumns; primeCol++)
		{
			if(primeMatrix[primeRow + nOfRows*primeCol])
			{
				break;
			}
		}
		/* star the primed zero */
		newStarMatrix[primeRow + nOfRows*primeCol] = true;
		/* find starred zero in current column */
		starCol = primeCol;
		for(starRow=0; starRow<nOfRows; starRow++)
		{
			if(starMatrix[starRow + nOfRows*starCol])
			{
				break;
			}
		}
	}
	/* use temporary copy as new starMatrix */
	/* delete all primes, uncover all rows */
	for(size_t n=0; n<nOfElements; n++)
	{
		primeMatrix[n] = false;
		starMatrix[n]  = newStarMatrix[n];
	}
	for(size_t n=0; n<nOfRows; n++)
	{
		coveredRows[n] = false;
	}
	/* move to step 2a */
	step2a(assignment, nOfRows, nOfColumns, minDim);
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------
template <class T>
void AssignmentProblemSolver<T>::step5(int *assignment, size_t nOfRows, size_t nOfColumns, size_t minDim)
{
	/* find smallest uncovered element h */
	T h = numeric_limits<T>::max();
	for(size_t row=0; row<nOfRows; row++)
	{
		if(!coveredRows[row])
		{
			for(size_t col=0; col<nOfColumns; col++)
			{
				if(!coveredColumns[col])
				{
					T value = distMatrix[row + nOfRows*col];
					if(value < h)
					{
						h = value;
					}
				}
			}
		}
	}
	/* add h to each covered row */
	for(size_t row=0; row<nOfRows; row++)
	{
		if(coveredRows[row])
		{
			for(size_t col=0; col<nOfColumns; col++)
			{
				distMatrix[row + nOfRows*col] += h;
			}
		}
	}
	/* subtract h from each uncovered column */
	for(size_t col=0; col<nOfColumns; col++)
	{
		if(!coveredColumns[col])
		{
			for(size_t row=0; row<nOfRows; row++)
			{
				distMatrix[row + nOfRows*col] -= h;
			}
		}
	}
	/* move to step 3 */
	step3(assignment, nOfRows, nOfColumns, minDim);
}


// --------------------------------------------------------------------------
// Computes a suboptimal solution. Good for cases without forbidden assignments.
// --------------------------------------------------------------------------
template <class T>
void AssignmentProblemSolver<T>::assignmentsuboptimal2(int *assignment, T *cost, T *distMatrixIn, size_t nOfRows, size_t nOfColumns)
{
	/* make working copy of distance Matrix */
	const size_t nOfElements = nOfRows * nOfColumns;
	T *distMatrix            = (T *)malloc(nOfElements * sizeof(T));
	for(size_t n=0; n<nOfElements; n++)
	{
		distMatrix[n] = distMatrixIn[n];
	}

	/* initialization */
	*cost = 0;
	for(size_t row=0; row<nOfRows; row++)
	{
		assignment[row] = -1;
	}

	size_t tmpRow, tmpCol;
	/* recursively search for the minimum element and do the assignment */
	while(true)
	{
		/* find minimum distance observation-to-track pair */
		T minValue = numeric_limits<T>::max();
		for(size_t row=0; row<nOfRows; row++)
			for(size_t col=0; col<nOfColumns; col++)
			{
				T value = distMatrix[row + nOfRows*col];
				if(value!=numeric_limits<T>::max() && (value < minValue))
				{
					minValue = value;
					tmpRow   = row;
					tmpCol   = col;
				}
			}

		if(minValue!=numeric_limits<T>::max())
		{
			assignment[tmpRow] = tmpCol;
			*cost += minValue;
			for(size_t n=0; n<nOfRows; n++)
			{
				distMatrix[n + nOfRows*tmpCol] = numeric_limits<T>::max();
			}
			for(size_t n=0; n<nOfColumns; n++)
			{
				distMatrix[tmpRow + nOfRows*n] = numeric_limits<T>::max();
			}
		}
		else
			break;

	} /* while(true) */

	free(distMatrix);
}
// --------------------------------------------------------------------------
// Computes a suboptimal solution. Good for cases with many forbidden assignments.
// --------------------------------------------------------------------------
template <class T>
void AssignmentProblemSolver<T>::assignmentsuboptimal1(int *assignment, T *cost, T *distMatrixIn, size_t nOfRows, size_t nOfColumns)
{
	size_t tmpRow, tmpCol;
	int *nOfValidObservations, *nOfValidTracks;
	T minValue;

	/* make working copy of distance Matrix */
	const size_t nOfElements = nOfRows * nOfColumns;
	T* distMatrix            = (T *)malloc(nOfElements * sizeof(T));
	for(size_t n=0; n<nOfElements; n++)
	{
		distMatrix[n] = distMatrixIn[n];
	}
	/* initialization */
	*cost = 0;

	for(size_t row=0; row<nOfRows; row++)
	{
		assignment[row] = -1;
	}

	/* allocate memory */
	nOfValidObservations  = (int *)calloc(nOfRows,    sizeof(int));
	nOfValidTracks        = (int *)calloc(nOfColumns, sizeof(int));

	/* compute number of validations */
	bool infiniteValueFound = false;
	bool finiteValueFound  = false;
	for(size_t row=0; row<nOfRows; row++)
	{
		for(size_t col=0; col<nOfColumns; col++)
		{
			if(distMatrix[row + nOfRows*col]!=numeric_limits<T>::max())
			{
				nOfValidTracks[col]       += 1;
				nOfValidObservations[row] += 1;
				finiteValueFound = true;
			}
			else
				infiniteValueFound = true;
		}
	}

	if(infiniteValueFound)
	{
		if(!finiteValueFound)
		{
			/* free allocated memory */
			free(nOfValidObservations);
			free(nOfValidTracks);
			free(distMatrix);
			return;
		}
		bool repeatSteps = true;

		while(repeatSteps)
		{
			repeatSteps = false;

			/* step 1: reject assignments of multiply validated tracks to singly validated observations		 */
			for(size_t col=0; col<nOfColumns; col++)
			{
				bool singleValidationFound = false;
				for(size_t row=0; row<nOfRows; row++)
					if(distMatrix[row + nOfRows*col]!=numeric_limits<T>::max() && (nOfValidObservations[row] == 1))
					{
						singleValidationFound = true;
						break;
					}

				if(singleValidationFound)
				{
					for(size_t row=0; row<nOfRows; row++)
						if((nOfValidObservations[row] > 1) && distMatrix[row + nOfRows*col]!=numeric_limits<T>::max())
						{
							distMatrix[row + nOfRows*col] = numeric_limits<T>::max();
							nOfValidObservations[row] -= 1;
							nOfValidTracks[col]       -= 1;
							repeatSteps = true;
						}
				}
			}

			/* step 2: reject assignments of multiply validated observations to singly validated tracks */
			if(nOfColumns > 1)
			{
				for(size_t row=0; row<nOfRows; row++)
				{
					bool singleValidationFound = false;
					for(size_t col=0; col<nOfColumns; col++)
					{
						if(distMatrix[row + nOfRows*col]!=numeric_limits<T>::max() && (nOfValidTracks[col] == 1))
						{
							singleValidationFound = true;
							break;
						}
					}

					if(singleValidationFound)
					{
						for(size_t col=0; col<nOfColumns; col++)
						{
							if((nOfValidTracks[col] > 1) && distMatrix[row + nOfRows*col]!=numeric_limits<T>::max())
							{
								distMatrix[row + nOfRows*col] = numeric_limits<T>::max();
								nOfValidObservations[row] -= 1;
								nOfValidTracks[col]       -= 1;
								repeatSteps = true;
							}
						}
					}
				}
			}
		} /* while(repeatSteps) */

		/* for each multiply validated track that validates only with singly validated  */
		/* observations, choose the observation with minimum distance */
		for(size_t row=0; row<nOfRows; row++)
		{
			if(nOfValidObservations[row] > 1)
			{
				bool allSinglyValidated = true;
				minValue = numeric_limits<T>::max();
				for(size_t col=0; col<nOfColumns; col++)
				{
					T value = distMatrix[row + nOfRows*col];
					if(value!=numeric_limits<T>::max())
					{
						if(nOfValidTracks[col] > 1)
						{
							allSinglyValidated = false;
							break;
						}
						else if((nOfValidTracks[col] == 1) && (value < minValue))
						{
							tmpCol   = col;
							minValue = value;
						}
					}
				}

				if(allSinglyValidated)
				{
					assignment[row] = tmpCol;
					*cost += minValue;
					for(size_t n=0; n<nOfRows; n++)
					{
						distMatrix[n + nOfRows*tmpCol] = numeric_limits<T>::max();
					}
					for(size_t n=0; n<nOfColumns; n++)
					{
						distMatrix[row + nOfRows*n] = numeric_limits<T>::max();
					}
				}
			}
		}

		/* for each multiply validated observation that validates only with singly validated  */
		/* track, choose the track with minimum distance */
		for(size_t col=0; col<nOfColumns; col++)
		{
			if(nOfValidTracks[col] > 1)
			{
				bool allSinglyValidated = true;
				minValue = numeric_limits<T>::max();
				for(size_t row=0; row<nOfRows; row++)
				{
					T value = distMatrix[row + nOfRows*col];
					if(value!=numeric_limits<T>::max())
					{
						if(nOfValidObservations[row] > 1)
						{
							allSinglyValidated = false;
							break;
						}
						else if((nOfValidObservations[row] == 1) && (value < minValue))
						{
							tmpRow   = row;
							minValue = value;
						}
					}
				}

				if(allSinglyValidated)
				{
					assignment[tmpRow] = col;
					*cost += minValue;
					for(size_t n=0; n<nOfRows; n++)
						distMatrix[n + nOfRows*col] = numeric_limits<T>::max();
					for(size_t n=0; n<nOfColumns; n++)
						distMatrix[tmpRow + nOfRows*n] = numeric_limits<T>::max();
				}
			}
		}
	} /* if(infiniteValueFound) */


	/* now, recursively search for the minimum element and do the assignment */
	while(true)
	{
		/* find minimum distance observation-to-track pair */
		minValue = numeric_limits<T>::max();
		for(size_t row=0; row<nOfRows; row++)
			for(size_t col=0; col<nOfColumns; col++)
			{
				T value = distMatrix[row + nOfRows*col];
				if(value!=numeric_limits<T>::max() && (value < minValue))
				{
					minValue = value;
					tmpRow   = row;
					tmpCol   = col;
				}
			}

		if(minValue!=numeric_limits<T>::max())
		{
			assignment[tmpRow] = tmpCol;
			*cost += minValue;
			for(size_t n=0; n<nOfRows; n++)
				distMatrix[n + nOfRows*tmpCol] = numeric_limits<T>::max();
			for(size_t n=0; n<nOfColumns; n++)
				distMatrix[tmpRow + nOfRows*n] = numeric_limits<T>::max();
		}
		else
			break;

	} /* while(true) */

	/* free allocated memory */
	free(nOfValidObservations);
	free(nOfValidTracks);
	free(distMatrix);
}
/*
// --------------------------------------------------------------------------
// Usage example
// --------------------------------------------------------------------------
void main(void)
{
// Matrix size
int N=8; // tracks
int M=9; // detects
// Random numbers generator initialization
srand (time(NULL));
// Distance matrix N-th track to M-th detect.
vector< vector<double> > Cost(N,vector<double>(M));
// Fill matrix with random values
for(int i=0; i<N; i++)
{
for(int j=0; j<M; j++)
{
Cost[i][j] = (double)(rand()%1000)/1000.0;
std::cout << Cost[i][j] << "\t";
}
std::cout << std::endl;
}

AssignmentProblemSolver APS;

vector<int> Assignment;

cout << APS.Solve(Cost,Assignment) << endl;

// Output the result
for(int x=0; x<N; x++)
{
std::cout << x << ":" << Assignment[x] << "\t";
}

getchar();
}
*/
// --------------------------------------------------------------------------
//
template class AssignmentProblemSolver<double>;
template class AssignmentProblemSolver<float>;
template class AssignmentProblemSolver<unsigned int>;
