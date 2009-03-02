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

  void AddNewColumn(double cost, double lower_bound, double upper_bound,
                    int machine, int index, const vector<int>& tasks,
                    VariableGeracaoColunasHash* vHash, OPT_LP* lp);

  void AddStabilizationColumns(const ProblemData& p,
                               const vector<double>& relaxed_dual_values,
                               VariableGeracaoColunasHash* vHash, OPT_LP* lp);

  void AddIntegerSolutionToModel(const ProblemData& p,
                                 const vector<double>& relaxed_dual_values,
                                 VariableGeracaoColunasHash* vHash, OPT_LP* lp);

  /********************************************************************
  **                    CONSTRAINT CREATION                          **
  *********************************************************************/
  void AddSumAssignmentsPerTaskEqualsOneConstraints(
      const ProblemData& pd, ConstraintGeracaoColunasHash* cHash, OPT_LP* lp);
  void AddSumAssignmentsPerMachineAtMostOneConstraints(
      const ProblemData& pd, ConstraintGeracaoColunasHash* cHash, OPT_LP* lp);

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

  void RemoveExcessColumns(const ProblemData& pd,
                           int num_columns_to_remove,
                           VariableGeracaoColunasHash* vHash,
                           OPT_LP* lp);

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
  enum FixingSense {
    FIX_ON_ZERO = 0,
    FIX_ON_ONE
  };

  void SetAndStoreFixedVariables(
      FixingSense sense, const ProblemData& p, int fixed_task,
      vector<vector<short> >* fixed, vector<short> before_fixing);

  bool ShouldRemoveColumnWhenFixing(
      FixingSense sense, const VariableGeracaoColunas& var,
      int fixed_machine, int fixed_task);

  void FixVariableAndContinueBB(
    FixingSense fixing_sense, int fixed_machine, int fixed_task,
    int num_nodes_limit, double lower_bound, const ProblemData& p,
    double lower_bound, OPT_LP* lp, double* best_solution_value,
    ProblemSolution* best_solution, vector<vector<short> >* fixed_vars,
    int* num_columns, int depth, double* pct_tree_solved,
    int* num_visited_nodes, STATUS_BB* status);

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
  int column_count_;
  static int kMaxNumberColumns_ = 40000;
  static int kNumColumnsToRemove_ = 10000;
  static int kMaxDepthFixingVars_ = 4;
  static int kNumberOfLookupsFixingVars_[kMaxDepthFixingVars_] = { 10, 8, 6, 4 };
};
