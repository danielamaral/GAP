#include "SolverFormulacaoPadrao.h"

#include <iostream>

#include "opt_cplex.h"


SolverFormulacaoPadrao::SolverFormulacaoPadrao(ProblemData* problem_data) : Solver(problem_data)
{
    lp_ = new OPT_CPLEX;
}

SolverFormulacaoPadrao::~SolverFormulacaoPadrao()
{
    if (lp_ != NULL)
        delete lp_;
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
                OPT_COL col(OPT_COL::VAR_INTEGRAL, coef, 0, 1, (char*) var.ToString().c_str());
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
        int nnz = 1;
        OPT_ROW row(nnz, OPT_ROW::LESS, rhs, (char*) cons.ToString().c_str());

        // adds each variable
        for (int j = 0; j < problem_data_->n(); ++j) {
            var.reset();
            var.set_type(VariableFormulacaoPadrao::X_ij);
            var.set_machine(i);
            var.set_task(j);
            VariableFormulacaoPadraoHash::iterator vit = vHash.find(var);
            if (vit == vHash.end()) {
                assert(FALSE);
            } else {
                double consumes = problem_data_->consume(i, j);
                row.insert(vit->second, consumes);
            }
        }
        ++num_cons;
    }

    return num_cons;
}

void SolverFormulacaoPadrao::CreateConsOneMachinePerTask() {
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
        int nnz = 1;
        OPT_ROW row(nnz, OPT_ROW::EQUAL, rhs, (char*) cons.ToString().c_str());

        // adds each variable
        for (int i = 0; i < problem_data_->m(); ++i) {
            var.reset();
            var.set_type(VariableFormulacaoPadrao::X_ij);
            var.set_machine(i);
            var.set_task(j);
            VariableFormulacaoPadraoHash::iterator vit = vHash.find(var);
            if (vit == vHash.end()) {
                assert(FALSE);
            } else {
                row.insert(vit->second, 1.0);
            }
        }
        ++num_cons;
    }
    return num_cons;
}

int SolverFormulacaoPadrao::solve() {
    /** creates the variables */
    CreateVarTaskAssignment();

    /** creates the constraints */
    CreateConsMachineCapacity();
    CreateConsOneMachinePerTask();

    /** sets lp parameters */
    lp_->writeProbLP("SolverFormulacaoPadrao");
    lp_->setMIPEmphasis(4);
    lp_->setMIPScreenLog(4); 
    lp_->setMIPRelTol(0.01);
    lp_->setPolishTime(30);
    lp_->setTimeLimit(600);

    /** solves the problem */
	int status = lp_->optimize(METHOD_MIP);

    char statusStr[100];
    lp_->getStatStr(status, statusStr);

    LOG(INFO) << " " << std::endl;
    LOG(INFO) << "Solution => status = " << statusStr << std::endl;

    if(status == OPTSTAT_MIPOPTIMAL || status == OPTSTAT_FEASIBLE) {
        LOG(INFO) << "           value  = " << lp_->getObjVal() << std::endl;
        LOG(INFO) << "           gap    = " << lp_->getMIPGap() * 100.0 << std::endl;
        //double* x = new double[lp_->getNumCols()];
        //leValorVariaveis(x);
        //delete x;
    }
}

void SolverFormulacaoPadrao::generate_solution(ProblemSolution* ps) {
}