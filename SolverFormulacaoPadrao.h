#pragma once

#include "solver.h"
#include "opt_lp.h"
#include "VariableFormulacaoPadrao.h"
#include "ConstraintFormulacaoPadrao.h"

#define LOG(INFO) std::cerr

class SolverFormulacaoPadrao : public Solver {
public:
    SolverFormulacaoPadrao(ProblemData* problem_data);
    ~SolverFormulacaoPadrao();
    int solve();
    void generate_solution(ProblemSolution* ps);
private:
    /********************************************************************
    **                     VARIABLE CREATION                           **
    *********************************************************************/
    /** Creates the X variable */
    int CreateVarTaskAssignment();

    /********************************************************************
    **                    CONSTRAINT CREATION                          **
    *********************************************************************/

    /** Creates the constraint of maximum machine capacity */
    int CreateConsMachineCapacity();

    /** Creates the constraint of only one machine assigned per task */
    int CreateConsOneMachinePerTask();

    /** The optimizer and the status variables */
    OPT_LP* lp_;
    OPTSTAT lp_status_;
    char lp_status_str_[120];

    /** Hash which associates the column number with the Variable object. */
    VariableFormulacaoPadraoHash vHash;

    /** Hash which associates the row number with the Constraint object. */
    ConstraintFormulacaoPadraoHash cHash;
};
