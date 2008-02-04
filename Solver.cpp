#include "Solver.h"
#include "ProblemData.h"
#include <stdio.h>

Solver::Solver(ProblemData* aProblemData)
{
   problem_data_ = aProblemData;
}

Solver::~Solver()
{
   problem_data_ = NULL;
}