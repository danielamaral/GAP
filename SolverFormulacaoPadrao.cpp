#include "SolverFormulacaoPadrao.h"

#include <iostream>
#include <cassert>

#include "logging.h"
#include "opt_cplex.h"
#include "ProblemData.h"

SolverFormulacaoPadrao::SolverFormulacaoPadrao(ProblemData* problem_data):
    VnsSolver(problem_data) {
    lp_ = new OPT_CPLEX;
    log_.open((wchar_t*)"log", std::ofstream::out);
}

SolverFormulacaoPadrao::~SolverFormulacaoPadrao() {
    if (lp_ != NULL)
        delete lp_;
    log_.close();
}

/* static */ int SolverFormulacaoPadrao::CreateVarTaskAssignment(
  const ProblemData& problem,
  const OPT_COL::VARTYPE variable_type,
  VariableFormulacaoPadraoHash* vHash,
  OPT_LP* lp) {
  int num_var = 0;
  VariableFormulacaoPadrao var;

  // for every machine
  for (int i = 0; i < problem.num_machines(); ++i) {
    // for every task
    for (int j = 0; j < problem.num_tasks(); ++j) {
      var.reset();
      var.set_type(VariableFormulacaoPadrao::X_ij);
      var.set_machine(i);
      var.set_task(j);
      if (vHash->find(var) == vHash->end()) {
        // finds the cost
        double coef = problem.cost(i, j);

        // inserts the variable
        int var_index = lp->getNumCols();
        vHash->insert(VariableFormulacaoPadraoIntPair(var, var_index));
        OPT_COL col(variable_type, coef, 0.0, 1.0, (char*) var.ToString().c_str());
        lp->newCol(col);
        ++num_var;
      }
    }
  }
  return num_var;
}

/* static */ int SolverFormulacaoPadrao::CreateConsMachineCapacity(
  const ProblemData& problem,
  VariableFormulacaoPadraoHash* vHash,
  ConstraintFormulacaoPadraoHash* cHash,
  OPT_LP* lp) {
  int num_cons = 0;
  // constraint
  ConstraintFormulacaoPadrao cons;
  // variable
  VariableFormulacaoPadrao var;

  for (int i = 0; i < problem.num_machines(); ++i) {
    double rhs = problem.capacity(i);
    // creates the constraint
    cons.reset();
    cons.set_type(ConstraintFormulacaoPadrao::C_MACHINE_CAPACITY);
    cons.set_machine(i);

    // skips constraints already inserted
    if (cHash->find(cons) != cHash->end()) {
      continue;
    }

    (*cHash)[cons] = lp->getNumRows();
    int nnz = problem.num_tasks();
    OPT_ROW row(nnz, OPT_ROW::LESS, rhs, (char*) cons.ToString().c_str());

    // adds each variable
    for (int j = 0; j < problem.num_tasks(); ++j) {
      var.reset();
      var.set_type(VariableFormulacaoPadrao::X_ij);
      var.set_machine(i);
      var.set_task(j);
      VariableFormulacaoPadraoHash::iterator vit = vHash->find(var);

      assert(vit != vHash->end());

      double consumes = problem.consume(i, j);
      row.insert(vit->second, consumes);
    }

    lp->addRow(row);
    ++num_cons;
  }

  return num_cons;
}

/* static */ int SolverFormulacaoPadrao::CreateConsOneMachinePerTask(
  const ProblemData& problem,
  VariableFormulacaoPadraoHash* vHash,
  ConstraintFormulacaoPadraoHash* cHash,
  OPT_LP* lp) {
  int num_cons = 0;
  // constraint
  ConstraintFormulacaoPadrao cons;
  // variable
  VariableFormulacaoPadrao var;

  for (int j = 0; j < problem.num_tasks(); ++j) {
    double rhs = 1.0;
    // creates the constraint
    cons.reset();
    cons.set_type(ConstraintFormulacaoPadrao::C_ONE_MACHINE_PER_TASK);
    cons.set_task(j);

    // skips constraints already inserted
    if (cHash->find(cons) != cHash->end()) {
      continue;
    }

    (*cHash)[cons] = lp->getNumRows();
    int nnz = problem.num_machines();
    OPT_ROW row(nnz, OPT_ROW::EQUAL, rhs, (char*) cons.ToString().c_str());

    // adds each variable
    for (int i = 0; i < problem.num_machines(); ++i) {
      var.reset();
      var.set_type(VariableFormulacaoPadrao::X_ij);
      var.set_machine(i);
      var.set_task(j);
      VariableFormulacaoPadraoHash::iterator vit = vHash->find(var);

      assert(vit != vHash->end());

      row.insert(vit->second, 1.0);
    }
    lp->addRow(row);
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
	VariableFormulacaoPadraoHash::iterator vit = vHash_.find(var);
	assert(vit != vHash_.end());
	lp_->chgCoef(cons_row, vit->second, coef);
}

void SolverFormulacaoPadrao::ClearConsMaxAssignmentChangesEllipsoidal() {
  ConstraintFormulacaoPadrao cons;
  cons.set_type(ConstraintFormulacaoPadrao::C_MAX_ASSIGNMENT_CHANGES_ELLIPSOIDAL);
  ConstraintFormulacaoPadraoHash::iterator cit = cHash_.find(cons);
	if (cit != cHash_.end()) {
		int cons_row = cit->second;
        for (int i = 0; i < lp_->getNumCols(); ++i) {
            lp_->chgCoef(cons_row, i, 0.0);
        }
        lp_->chgRHS(cons_row, 0.0);
	}
}

int SolverFormulacaoPadrao::AddEllipsoidalConstraint(
	const vector<ProblemSolution*>& x, OPT_ROW::ROWSENSE constraint_sense, int F) {
	int RHS = problem_data_->n() * x.size() - F;

  // creates the constraint
  ConstraintFormulacaoPadrao cons;
  cons.set_type(ConstraintFormulacaoPadrao::C_MAX_ASSIGNMENT_CHANGES_ELLIPSOIDAL);
  ConstraintFormulacaoPadraoHash::iterator cit = cHash_.find(cons);
  int cons_row;
  if (cit != cHash_.end()) {
    // the constraint already exists, zero it
    cons_row = cit->second;
	  ClearConsMaxAssignmentChangesEllipsoidal();
    lp_->chgSense(cons_row, constraint_sense);
    lp_->chgRHS(cons_row, RHS);
  } else {
    cons_row = lp_->getNumRows();
    cHash_[cons] = lp_->getNumRows();
    int nnz = problem_data_->n() * 2;
    OPT_ROW row(nnz, constraint_sense, RHS, (char*) cons.ToString().c_str());
    lp_->addRow(row);
  }

  // for each task
  for (int task = 0; task < problem_data_->n(); ++task) {
    // counts the weight of every machine assignment
    map<int, int> machine_to_counts;
    for (int sol = 0; sol < x.size(); ++sol) {
      machine_to_counts[x[sol]->assignment(task)] += 1;
    }
    // adds to the model
    for (map<int, int>::iterator it = machine_to_counts.begin();
      it != machine_to_counts.end(); ++it) {
      int machine = it->first;
      int count = it->second;
      SetVariable(cons_row, task, machine, count);
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

void SolverFormulacaoPadrao::ReverseConstraint(int cons_row, int rhs) {
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
	OPT_ROW row(nnz, OPT_ROW::GREATER, rhs,
              (char*) "IntegerSolutionStrongCuttingPlane");
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
  GenerateSolution(lp_, &vHash_, &cHash_, sol);
}

/* static */ void SolverFormulacaoPadrao::GenerateSolution(
  OPT_LP* lp,
  VariableFormulacaoPadraoHash* vHash,
  ConstraintFormulacaoPadraoHash* cHash,
  ProblemSolution* sol) {

  // Loads the solution into the Variable hash and constructs the
  // ProblemSolution
  double* x = new double[lp->getNumCols()];
  lp->getX(x);

  //cout << "loading solution!!!" << endl;
  int cont = 0;
  for (VariableFormulacaoPadraoHash::iterator vit = vHash->begin();
       vit != vHash->end(); ++vit) {
    // TODO(daniel): fix this ugly workaround
    VariableFormulacaoPadrao& v =
      const_cast<VariableFormulacaoPadrao&>(vit->first);
    v.set_value(x[vit->second]);
    // assigned variable
    if (x[vit->second] > Globals::EPS()) {
      ++cont;
      //cout << "assigning task " << vit->first.task() << " to machine "
      //     << vit->first.machine() << " (" << x[vit->second] << ")" << endl;
      sol->set_assignment(vit->first.task(), vit->first.machine());
    }
  }
  //cout << "cont = " << cont << endl;
  assert(sol->IsValid());
}

void SolverFormulacaoPadrao::Init(const SolverOptions& options) {
  /** creates the problem */
  lp_->createLP("FormulacaoPadrao", OPTSENSE_MINIMIZE, PROB_MIP);
  if (VLOG_IS_ON(6)) {
    lp_->setMIPScreenLog(2);
  }

  lp_->setMIPRelTol(0.00);
	lp_->setMIPAbsTol(0.00);
  
  // Maximum 2.8 gigs, store the rest on disk (uncompressed).
  lp_->setWorkMem(3000);
  lp_->setTreLim(20000);
  lp_->setNodeFileInd(2);
  if (problem_data_->num_tasks() >= 1600) {
    // Precisamos evitar esgotar a memória.
    lp_->setNodeSel(0);
    lp_->setVarSel(3);  // Strong branching.
    lp_->setMemoryEmphasis(true);
  }

  /** creates the variables */
  CreateVarTaskAssignment(*problem_data_, OPT_COL::VAR_BINARY, &vHash_, lp_);

  /** creates the constraints */
  CreateConsMachineCapacity(*problem_data_, &vHash_, &cHash_, lp_);
  CreateConsOneMachinePerTask(*problem_data_, &vHash_, &cHash_, lp_);
}

/* static */ void SolverFormulacaoPadrao::GetRelaxedDualValues(
    const ProblemData& p,
    vector<double>* dual) {
  OPT_LP* lp = new OPT_CPLEX;
  VariableFormulacaoPadraoHash vHash;
  ConstraintFormulacaoPadraoHash cHash;

  lp->createLP("FormulacaoPadraoLP", OPTSENSE_MINIMIZE, PROB_LP);
  CreateVarTaskAssignment(p, OPT_COL::VAR_CONTINUOUS, &vHash, lp);
  CreateConsMachineCapacity(p, &vHash, &cHash, lp);
  CreateConsOneMachinePerTask(p, &vHash, &cHash, lp);

  lp->optimize(METHOD_DUAL);

  dual->resize(lp->getNumRows());
  lp->getPi(&(*dual)[0]);
  delete lp;
}

/* static */ OPTSTAT SolverFormulacaoPadrao::GetFirstIntegerSolution(
  const ProblemData& p,
  ProblemSolution* sol) {
  
  OPT_LP* lp = new OPT_CPLEX;
  VariableFormulacaoPadraoHash vHash;
  ConstraintFormulacaoPadraoHash cHash;

  lp->createLP("FormulacaoPadraoFirstInteger", OPTSENSE_MINIMIZE, PROB_MIP);
  CreateVarTaskAssignment(p, OPT_COL::VAR_BINARY, &vHash, lp);
  CreateConsMachineCapacity(p, &vHash, &cHash, lp);
  CreateConsOneMachinePerTask(p, &vHash, &cHash, lp);

  lp->setNumIntSols(1);
  lp->optimize(METHOD_MIP);

  GenerateSolution(lp, &vHash, &cHash, sol);
  OPTSTAT status = lp->getStat();
  delete lp;
  return status;
}

/* static */ OPTSTAT SolverFormulacaoPadrao::GetIntegerSolutionWithTimeLimit(
  const ProblemData& p,
  int time_limit,
  ProblemSolution* sol) {
  
  OPT_LP* lp = new OPT_CPLEX;
  VariableFormulacaoPadraoHash vHash;
  ConstraintFormulacaoPadraoHash cHash;

  lp->createLP("FormulacaoPadraoFirstInteger", OPTSENSE_MINIMIZE, PROB_MIP);
  CreateVarTaskAssignment(p, OPT_COL::VAR_BINARY, &vHash, lp);
  CreateConsMachineCapacity(p, &vHash, &cHash, lp);
  CreateConsOneMachinePerTask(p, &vHash, &cHash, lp);

  //lp->setMIPScreenLog(3);
  // Maximum 1.5 gigs, store the rest on disk (uncompressed).
  lp->setWorkMem(1100);
  lp->setTreLim(1500);
  lp->setNodeFileInd(2);

  if (time_limit > 0)
    lp->setTimeLimit(time_limit);
  lp->optimize(METHOD_MIP);

  if (lp->getStat() != OPTSTAT_FEASIBLE) {
    lp->setNumIntSols(1);
    lp->optimize(METHOD_MIP);
  }

  GenerateSolution(lp, &vHash, &cHash, sol);
  OPTSTAT status = lp->getStat();
  delete lp;
  return status;
}

int SolverFormulacaoPadrao::Solve(const SolverOptions& options,
                                  SolverStatus* output_status) {
  int status = SolveTLAndUB(options.max_time(),
                            options.cut_off_value(), options.only_first_solution());
  if (output_status != NULL) {
    output_status->status = status;
    output_status->gap_absolute = GetGapAbsolute();
    output_status->gap_relative = GetGapRelative();
    if (status == OPTSTAT_FEASIBLE || status == OPTSTAT_MIPOPTIMAL)
	    GenerateSolution(&output_status->final_sol);
  }
  return status;
}

int SolverFormulacaoPadrao::SolveTLAndUB(int time_limit, double upper_bound, bool first) {
	/** sets lp parameters */
  lp_->writeProbLP("SolverFormulacaoPadrao");
	//lp_->setVarSel(3);
  //cout << "Time: " << time_limit;
  if (time_limit > 0)
	  lp_->setTimeLimit(time_limit);

	if (upper_bound < Globals::Infinity())
		lp_->setMIPCutOff(upper_bound);

    if (first)
        lp_->setNumIntSols(1);
    else
        lp_->setNumIntSols(0);  // maximum number

  /** solves the problem */
	return lp_->optimize(METHOD_MIP);
}

double SolverFormulacaoPadrao::GetGapRelative() {
	return lp_->getMIPRelGap();
}

double SolverFormulacaoPadrao::GetGapAbsolute() {
	return lp_->getMIPAbsGap();
}