#pragma once

#include <fstream>
#include <sstream>
#include "solver.h"
#include "opt_lp.h"
#include "ConstraintFormulacaoPadrao.h"
#include "Globals.h"
#include "ProblemSolution.h"
#include "VariableFormulacaoPadrao.h"

class SolverStatus {
public:
	SolverStatus() : status(-1), final_sol(Globals::instance()),
		gap_relative(0.0), gap_absolute(0.0), time (0.0) { }

	string ToString() {
		stringstream ss;
		ss << final_sol.cost() << " ";
		if (str_status != "heuristic")
			ss << gap_relative << " " << gap_absolute << " ";
		ss << time << " " << str_status;
		return ss.str();
	}
	int status;
	string str_status;

	ProblemSolution final_sol;
	
	double gap_relative;
	double gap_absolute;

	double time;
};

class SolverFormulacaoPadrao : public Solver {
public:
    SolverFormulacaoPadrao(ProblemData* problem_data);
    ~SolverFormulacaoPadrao();
    /// Solves the problem
    int Solve() { return Solve(60); }
    int Solve(int time_limit);
	void Solve(int time_limit, SolverStatus* status);
	
	int SolveTLAndUB(int time_limit, double upper_bound, bool first = false);
	void SolveTLAndUB(int time_limit, double upper_bound, bool first, SolverStatus* status);

	double GetGapRelative();
	double GetGapAbsolute();
    /// Initializes the problem (creates variables and constraints)
    void Init();
    /// Generates a ProblemSolution from the X vector
    void GenerateSolution(ProblemSolution* ps);

    /********************************************************************
    **                 LOCAL SEARCH CONSTRAINTS                        **
    *********************************************************************/

    // Adds a constraint: delta(x, sol) <= num_changes
    int AddConsMaxAssignmentChanges(const ProblemSolution& sol, int num_changes);
    // Adds a constraint: delta(x, sol) >= num_changes
    int AddConsMinAssignmentChanges(const ProblemSolution& sol, int num_changes);
	// Adds a constraint that determines that one of the assignments with negative reduced cost
	// has to enter the basis: sum(X_ij with negative cost) >= 1
	int AddConsIntegerSolutionStrongCuttingPlane(const ProblemSolution& sol);
    // Removes the constraint <cons_row>
    void RemoveConstraint(int cons_row);
    void RemoveConstraint(int cons_row_begin, int cons_row_end); // range removal
    // Changes the sense of the constraint <cons_row> and updates the right hand side to <rhs>
    void ReverseConstraint(int cons_row, double rhs);
    
	int UpdateConsMaxAssignmentChangesEllipsoidal(
		const ProblemSolution& x1, const ProblemSolution& x2, int k);
	void ClearConsMaxAssignmentChangesEllipsoidal();

private:
    /********************************************************************
    **                     VARIABLE CREATION                           **
    *********************************************************************/
    /** Creates the X variable */
    int CreateVarTaskAssignment();

	// Sets the variable x_task,machine with a coeficient
	// 'coef' in constraint cons_row
	void SetVariable(int cons_row, int task, int machine, double coef);

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
    std::ofstream log;

    /** Hash which associates the column number with the Variable object. */
    VariableFormulacaoPadraoHash vHash;

    /** Hash which associates the row number with the Constraint object. */
    ConstraintFormulacaoPadraoHash cHash;
};
