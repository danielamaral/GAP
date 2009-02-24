#pragma once

#include <fstream>
#include <sstream>
#include "solver.h"
#include "opt_lp.h"
#include "ConstraintGeracaoColunas.h"
#include "Globals.h"
#include "ProblemSolution.h"
#include "VariableGeracaoColunas.h"

class SolverStatus;

class FixingCandidate {
public:
  FixingCandidate() : task(-1), machine(-1), value(0.0) {}
  bool operator<(const FixingCandidate& cand) {
    if (this->value != cand.value)
      return this->value > cand.value;
    if (this->task != cand.task)
      return this->task < cand.task;
    return this->machine < cand.machine;
  }

  int task, machine;
  double value;
};

class SolverGeracaoColunas : public Solver {
public:
  SolverGeracaoColunas(ProblemData* problem_data);
  ~SolverGeracaoColunas();

  /********************************************************************
  **                SOLVING AND RESULTS METHODS                      **
  *********************************************************************/

	void Solve(int time_limit, SolverStatus* status);	
	void SolveWithCutoff(int time_limit, double cutoff,
                       bool first, SolverStatus* status);
	STATUS_BB SolveWithCutoff(int time_limit, double cutoff, bool first = false);
	double GetGapRelative();
	double GetGapAbsolute();

  /// Generates a ProblemSolution from the X vector
  void GenerateSolution(ProblemSolution* ps);

  /********************************************************************
  **                 LOCAL SEARCH CONSTRAINTS                        **
  *********************************************************************/


private:
  /********************************************************************
  **                       UTILITY METHODS                           **
  *********************************************************************/

  /// Initializes the problem (creates variables and constraints)
  void SetUpCplexParams(OPT_LP* lp);

  /********************************************************************
  **             VARIABLE CREATION + STABILIZATION                   **
  *********************************************************************/

	// Sets the variable x_task,machine with a coeficient
	// 'coef' in constraint cons_row
	void SetVariable(int cons_row, int task, int machine, double coef);

  void AddIdentityColumns();
  void AddStabilizationColumns();
  void AddIntegerSolutionToModel();

  /********************************************************************
  **                    CONSTRAINT CREATION                          **
  *********************************************************************/
  void AddSumAssignmentsPerTaskEqualsOneConstraints(*problem_data_, lp);
  void AddSumAssignmentsPerMachineAtMostOneConstraints(*problem_data_, lp);

  /********************************************************************
  **                   COLUMN GENERATION METHODS                     **
  *********************************************************************/
  double GenerateColumnsWithStabilization(const ProblemData& p,
                                          OPT_LP* lp,
                                          int* num_columns,
                                          vector<vector<short> >* fixed_vars,
                                          double best_solution_value);
  double GenerateColumns(const ProblemData& p,
                         OPT_LP* lp,
                         int* num_columns,
                         vector<vector<short> >* fixed_vars,
                         double best_solution_value);

  /********************************************************************
  **                    BRANCH AND BOUND METHODS                     **
  *********************************************************************/
  STATUS_BB SetUpAndRunBranchAndBound(const ProblemData& p,
                                      int num_nodes_limit,
                                      double best_integer,
                                      OPT_LP* lp,
                                      ProblemSolution* integer_solution);

  /// Returns a lower bound
  double BB(const ProblemData& p,
            int num_nodes_limit,
            int depth,
            double lower_bound,
            OPT_LP* lp,
            int* best_integer,
            ProblemSolution* best_solution,
            vector<vector<short> >* fixed,
            int* num_visited_nodes,
            STATUS_BB* status);

  // Funções para fixar o valor de uma variável de alocação (Xij) e continuar
  // o branch and bound. Elas armazenam o estado, fazem as modificações
  // necessárias nas colunas e chamam o BB. Depois que o BB retorna, elas
  // retornam as colunas e a matriz 'fixed_vars' ao seu estado original.
  void FixVarOnZero(
    int fixed_machine, int fixed_task, int num_nodes_limit,
    double lower_bound, const ProblemData& p, double lower_bound, OPT_LP* lp,
    double* best_solution_value, ProblemSolution* best_solution,
    vector<vector<short> >* fixed_vars, int* num_columns, int depth,
    double* pct_tree_solved, int* num_visited_nodes, STATUS_BB* status);

  void FixVarOnOne(
    int fixed_machine, int fixed_task, int num_nodes_limit,
    double lower_bound, const ProblemData& p, double lower_bound, OPT_LP* lp,
    double* best_solution_value, ProblemSolution* best_solution,
    vector<vector<short> >* fixed_vars, int* num_columns, int depth,
    double* pct_tree_solved, int* num_visited_nodes, STATUS_BB* status);

  // Usa uma função heurística para selecionar variáveis candidatas
  // a serem fixadas. Depois pega essas candidatas e submete a um
  // rápido ("raso") BB para escolher a que gera o melhor lower bound.
  int SelectFixedVariable(const ProblemData& p, int num_vars_lookup,
                          vector<vector<double> >* x, OPT_LP* lp,
                          double* best_solution_value,
                          ProblemSolution* best_solution,
                          vector<vector<short> >* fixed_vars, int* num_columns,
                          int* fixed_machine, int* fixed_task);

  double EvaluateVariableToFix(double binary_var);

  /** Hash which associates the column number with the Variable object. */
  VariableGeracaoColunasHash vHash_;

  /** Hash which associates the row number with the Constraint object. */
  ConstraintGeracaoColunasHash cHash_;

  int num_pivo_;
  static int kMaxNumberColumns_ = 40000;
  static int kMaxDepthFixingVars_ = 4;
  static int kNumberOfLookupsFixingVars_[kMaxDepthFixingVars_] = { 10, 8, 6, 4 };
};
