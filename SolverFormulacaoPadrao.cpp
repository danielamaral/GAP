#include "SolverFormulacaoPadrao.h"

#include <iostream>
#include <cassert>

#include "opt_cplex.h"
#include "ProblemData.h"
#include "ProblemSolution.h"


SolverFormulacaoPadrao::SolverFormulacaoPadrao(ProblemData* problem_data) : Solver(problem_data)
{
    lp_ = new OPT_CPLEX;
    log.open((wchar_t*)"log", std::ofstream::out);
}

SolverFormulacaoPadrao::~SolverFormulacaoPadrao()
{
    if (lp_ != NULL)
        delete lp_;
    log.close();
}

int SolverFormulacaoPadrao::CreateVarTaskAssignment() {
    int num_var = 0;
    VariableFormulacaoPadrao var;

    // for every machine
    for (int i = 0; i < problem_data_->m(); ++i) {
        // for every task
        for (int j = 0; j < problem_data_->n(); ++j) {
            var.reset();
            var.set_type(VariableFormulacaoPadrao::X_ij);
            var.set_machine(i);
            var.set_task(j);
            if (vHash.find(var) == vHash.end()) {
                // finds the cost
                double coef = problem_data_->cost(i, j);

                // inserts the variable
                int var_index = lp_->getNumCols();
                vHash.insert(VariableFormulacaoPadraoIntPair(var, var_index));
                OPT_COL col(OPT_COL::VAR_BINARY, coef, 0.0, 1.0, (char*) var.ToString().c_str());
                lp_->newCol(col);
                ++num_var;
            }
        }
    }
    return num_var;
}

int SolverFormulacaoPadrao::CreateConsMachineCapacity() {
    int num_cons = 0;
    // constraint
    ConstraintFormulacaoPadrao cons;
    // variable
    VariableFormulacaoPadrao var;

    for (int i = 0; i < problem_data_->m(); ++i) {
        double rhs = problem_data_->capacity(i);
        // creates the constraint
        cons.reset();
        cons.set_type(ConstraintFormulacaoPadrao::C_MACHINE_CAPACITY);
        cons.set_machine(i);

        // skips constraints already inserted
        if (cHash.find(cons) != cHash.end()) {
            continue;
        }

        cHash[cons] = lp_->getNumRows();
        int nnz = problem_data_->n();
        OPT_ROW row(nnz, OPT_ROW::LESS, rhs, (char*) cons.ToString().c_str());

        // adds each variable
        for (int j = 0; j < problem_data_->n(); ++j) {
            var.reset();
            var.set_type(VariableFormulacaoPadrao::X_ij);
            var.set_machine(i);
            var.set_task(j);
            VariableFormulacaoPadraoHash::iterator vit = vHash.find(var);
            if (vit == vHash.end()) {
                assert(false);
            } else {
                double consumes = problem_data_->consume(i, j);
                row.insert(vit->second, consumes);
                log << " + " << consumes << " " << vit->first.ToString();
            }
        }

        log << " <= " << rhs;
        lp_->addRow(row);
        ++num_cons;
    }

    return num_cons;
}

int SolverFormulacaoPadrao::CreateConsOneMachinePerTask() {
    int num_cons = 0;
    // constraint
    ConstraintFormulacaoPadrao cons;
    // variable
    VariableFormulacaoPadrao var;

    for (int j = 0; j < problem_data_->n(); ++j) {
        double rhs = 1.0;
        // creates the constraint
        cons.reset();
        cons.set_type(ConstraintFormulacaoPadrao::C_ONE_MACHINE_PER_TASK);
        cons.set_task(j);

        // skips constraints already inserted
        if (cHash.find(cons) != cHash.end()) {
            continue;
        }

        cHash[cons] = lp_->getNumRows();
        int nnz = problem_data_->m();
        OPT_ROW row(nnz, OPT_ROW::EQUAL, rhs, (char*) cons.ToString().c_str());

        // adds each variable
        for (int i = 0; i < problem_data_->m(); ++i) {
            var.reset();
            var.set_type(VariableFormulacaoPadrao::X_ij);
            var.set_machine(i);
            var.set_task(j);
            VariableFormulacaoPadraoHash::iterator vit = vHash.find(var);
            assert(vit != vHash.end());
            log << "+ 1 " << vit->first.ToString();
            row.insert(vit->second, 1.0);
        }
        log << " = 1.0" << std::endl;
        lp_->addRow(row);
        ++num_cons;
    }
    return num_cons;
}

void SolverFormulacaoPadrao::SetVariable(int cons_row, int task, int machine, double coef) {
    // variable
    VariableFormulacaoPadrao var;
	var.reset();
	var.set_type(VariableFormulacaoPadrao::X_ij);
	var.set_task(task);
	var.set_machine(machine);
	VariableFormulacaoPadraoHash::iterator vit = vHash.find(var);
	assert(vit != vHash.end());
	lp_->chgCoef(cons_row, vit->second, coef);
}

void SolverFormulacaoPadrao::ClearConsMaxAssignmentChangesEllipsoidal() {
    ConstraintFormulacaoPadrao cons;
    cons.set_type(ConstraintFormulacaoPadrao::C_MAX_ASSIGNMENT_CHANGES_ELLIPSOIDAL);
    ConstraintFormulacaoPadraoHash::iterator cit = cHash.find(cons);
	if (cit != cHash.end()) {
		int cons_row = cit->second;
        for (int i = 0; i < lp_->getNumCols(); ++i) {
            lp_->chgCoef(cons_row, i, 0.0);
        }
        lp_->chgRHS(cons_row, 0.0);
	}
}

int SolverFormulacaoPadrao::UpdateConsMaxAssignmentChangesEllipsoidal(
	const ProblemSolution& x1, const ProblemSolution& x2, int k) {
	// calculates the alfa(x1, x2) - N + |intersection|
	int alfa = problem_data_->n();
	for (int i = 0; i < problem_data_->n(); ++i) {
		if (x1.assignment(i) == x2.assignment(i))
			++alfa;
	}
	int rhs = alfa - k;

    // creates the constraint
    ConstraintFormulacaoPadrao cons;
    cons.set_type(ConstraintFormulacaoPadrao::C_MAX_ASSIGNMENT_CHANGES_ELLIPSOIDAL);
    ConstraintFormulacaoPadraoHash::iterator cit = cHash.find(cons);
    int cons_row;
    if (cit != cHash.end()) {
        // the constraint already exists, zero it
        cons_row = cit->second;
		ClearConsMaxAssignmentChangesEllipsoidal();
        lp_->chgRHS(cons_row, rhs);
    } else {
        cons_row = lp_->getNumRows();
        cHash[cons] = lp_->getNumRows();
        int nnz = problem_data_->n() * 2;
        OPT_ROW row(nnz, OPT_ROW::GREATER, rhs, (char*) cons.ToString().c_str());
        lp_->addRow(row);
    }

    // for each task
    for (int i = 0; i < problem_data_->n(); ++i) {
		// if the task is in the intersection
		if (x1.assignment(i) == x2.assignment(i)) {
			SetVariable(cons_row, i, x1.assignment(i), 2.0);
		} else {  // adds both x1_ij and x2_ij
			SetVariable(cons_row, i, x1.assignment(i), 1.0);
			SetVariable(cons_row, i, x2.assignment(i), 1.0);
		}
    }

    return 1;
}

void SolverFormulacaoPadrao::RemoveConstraint(int cons_row) {
    // TODO(danielrocha) : a better way to do this
    lp_->delRows(cons_row, cons_row);
}

void SolverFormulacaoPadrao::RemoveConstraint(int cons_row_begin, int cons_row_end) {
    // TODO(danielrocha) : a better way to do this
    lp_->delRows(cons_row_begin, cons_row_end);
}

void SolverFormulacaoPadrao::ReverseConstraint(int cons_row, double rhs) {
    if (lp_->getSense(cons_row) == OPT_ROW::GREATER)
        lp_->chgSense(cons_row, OPT_ROW::LESS);
    else
        lp_->chgSense(cons_row, OPT_ROW::GREATER);
    lp_->chgRHS(cons_row, problem_data_->n() - rhs);
}

int SolverFormulacaoPadrao::AddConsMaxAssignmentChanges(const ProblemSolution& sol, int num_changes) {
	// the right hand side
	int rhs = problem_data_->n() - num_changes;

    int cons_row = lp_->getNumRows();
    int nnz = problem_data_->n();
    OPT_ROW row(nnz, OPT_ROW::GREATER, rhs, (char*) "ConsMaxAssignmentChanges");
    lp_->addRow(row);

    // for each task
    for (int i = 0; i < problem_data_->n(); ++i) {
		SetVariable(cons_row, i, sol.assignment(i), 1.0);
    }

    return cons_row;
}

int SolverFormulacaoPadrao::AddConsMinAssignmentChanges(const ProblemSolution& sol, int num_changes) {
	// the right hand side
	int rhs = problem_data_->n() - num_changes;

    int cons_row = lp_->getNumRows();
    int nnz = problem_data_->n();
    OPT_ROW row(nnz, OPT_ROW::LESS, rhs, (char*) "ConsMinAssignmentChanges");
    lp_->addRow(row);

    // for each task
    for (int i = 0; i < problem_data_->n(); ++i) {
		SetVariable(cons_row, i, sol.assignment(i), 1.0);
    }

    return cons_row;
}

int SolverFormulacaoPadrao::AddConsIntegerSolutionStrongCuttingPlane(const ProblemSolution& sol) {
	int rhs = 1;
	int cons_row = lp_->getNumRows();
	// All X_ij variables except for the ones already at the basis
	int nnz = problem_data_->n() * (problem_data_->m() - 1);
	OPT_ROW row(nnz, OPT_ROW::GREATER, rhs, (char*) "IntegerSolutionStrongCuttingPlane");
	lp_->addRow(row);

	// for each task
	for (int t = 0; t < problem_data_->n(); ++t) {
		// for each machine
		for (int m = 0; m < problem_data_->m(); ++m) {
			// If c_ij - c_ix < 0, add to the restriction
			if (problem_data_->cost(m, t) - problem_data_->cost(sol.assignment(t), t) < 0.0) {
				SetVariable(cons_row, t, m, 1.0);
			}
		}
	}

	return cons_row;
}
void SolverFormulacaoPadrao::GenerateSolution(ProblemSolution* sol) {
    // Loads the solution into the Variable hash and constructs the ProblemSolution
    double* x = new double[lp_->getNumCols()];
    lp_->getX(x);

    //cout << "loading solution!!!" << endl;
    int cont = 0;
    for (VariableFormulacaoPadraoHash::iterator vit = vHash.begin(); vit != vHash.end(); ++vit) {
        // TODO(daniel): fix this ugly workaround
        VariableFormulacaoPadrao& v = const_cast<VariableFormulacaoPadrao&>(vit->first);
        v.set_value(x[vit->second]);
        // assigned variable
        if (x[vit->second] > 1e-9) {
            ++cont;
            //cout << "assigning task " << vit->first.task() << " to machine " << vit->first.machine() << " (" << x[vit->second] << ")" << endl;
            sol->set_assignment(vit->first.task(), vit->first.machine());
        }
    }
    //cout << "cont = " << cont << endl;
    assert(sol->IsValid());
}

void SolverFormulacaoPadrao::Init() {
    /** creates the problem */
    lp_->createLP("FormulacaoPadrao", OPTSENSE_MINIMIZE, PROB_MIP);
    lp_->setMIPScreenLog(0);
    lp_->setMIPEmphasis(4);
    lp_->setMIPRelTol(0.00);

    /** creates the variables */
    CreateVarTaskAssignment();

    /** creates the constraints */
    CreateConsMachineCapacity();
    CreateConsOneMachinePerTask();
}

int SolverFormulacaoPadrao::SolveTLAndUB(int time_limit, double upper_bound, bool first) {
    lp_->setTimeLimit(time_limit);
    lp_->setMIPCutOff(upper_bound);
    if (first) {
        lp_->setNumIntSols(1);
    } else {
        lp_->setNumIntSols(0);  // maximum number
    }
    //lp_->writeProbLP("SolverFormulacaoPadrao");

    return lp_->optimize(METHOD_MIP);
}

int SolverFormulacaoPadrao::Solve(int time_limit, int polish_time) {
	/** sets lp parameters */
    lp_->writeProbLP("SolverFormulacaoPadrao");
    lp_->setTimeLimit(time_limit);

    /** solves the problem */
	int status = lp_->optimize(METHOD_MIP);

    char statusStr[100];
    lp_->getStatStr(status, statusStr);

    log << " " << std::endl;
    log << "Solution => status = " << statusStr << std::endl;

    if(status == OPTSTAT_MIPOPTIMAL || status == OPTSTAT_FEASIBLE) {
        log << "           value  = " << lp_->getObjVal() << std::endl;
        log << "           gap    = " << lp_->getMIPGap() * 100.0 << std::endl;
        //ProblemSolution sol(problem_data_);
        //GenerateSolution(&sol);
        //log << "Final solution:" << std::endl;
        //log << sol << std::endl;
        return 0;
    } else {
        return 1;
    }
}
