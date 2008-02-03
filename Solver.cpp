#include "Solver.h"
#include "ProblemData.h"
#include <stdio.h>

Solver::Solver(ProblemData *aProblemData)
{
   problemData = aProblemData;
}

Solver::~Solver()
{
   problemData = NULL;
}