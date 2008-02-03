#pragma once
#include "solver.h"

class SolverFormulacaoPadrao :
    public Solver
{
public:
    SolverFormulacaoPadrao(ProblemData* problem_data);
    ~SolverFormulacaoPadrao();
    int solve();
    void generate_solution(ProblemSolution* ps);
private:
    OPT_LP* lp_;
    OPTSTAT lp_status_;
    char lp_status_str_[120];

};
