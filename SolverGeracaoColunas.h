#pragma once

#include <fstream>
#include <sstream>
#include <vector>

#include "solver.h"
#include "opt_lp.h"
#include "ConstraintGeracaoColunas.h"
#include "Globals.h"
#include "ProblemSolution.h"
#include "VariableGeracaoColunas.h"

using namespace std;

class SolverStatus;

class FixingCandidate {
public:
  FixingCandidate() : task(-1), machine(-1), value(0.0) {}
  bool operator<(const FixingCandidate& cand) const {
    if (this->value != cand.value)
      return this->value < cand.value;
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

  int Solve();
	void Solve(int time_limit, SolverStatus* status);	
	void SolveWithCutoff(int time_limit, double cutoff,
                       bool first, SolverStatus* status);
	OPTSTAT SolveWithCutoff(int time_limit, double cutoff, bool first = false);
	double GetGapRelative();
	double GetGapAbsolute();

  /// Generates a ProblemSolution from the X vector
  void GenerateSolution(const ProblemData& p,
                        const VariableGeracaoColunasHash& vHash,
                        const OPT_LP& lp,
                        ProblemSolution* sol) const;

  /********************************************************************
  **                 LOCAL SEARCH CONSTRAINTS                        **
  *********************************************************************/


private:
  /********************************************************************
  **                       UTILITY METHODS                           **
  *********************************************************************/

  /// Initializes the problem (creates variables and constraints)
  void SetUpCplexParams(OPT_LP* lp);

  /// Generates a Xij matrix from the given fractionary solution
  void GetXijMatrix(const vector<double>& current_sol,
                    const VariableGeracaoColunasHash& vHash,
                    vector<vector<double> >* x) const;

  /********************************************************************
  **             VARIABLE CREATION + STABILIZATION                   **
  *********************************************************************/

  void AddNewColumn(double cost, double lower_bound, double upper_bound,
                    int machine, const vector<int>& tasks,
                    VariableGeracaoColunasHash* vHash, OPT_LP* lp);

  void AddNewColumn(const VariableGeracaoColunas& var,
                    double lower_bound, double upper_bound, 
                    VariableGeracaoColunasHash* vHash, OPT_LP* lp);

  void AddStabilizationColumnsWithCoeficients(
      const ProblemData& p, const vector<double>& relaxed_dual_values,
      VariableGeracaoColunasHash* vHash, OPT_LP* lp);

  void AddIntegerSolutionColumnsWithCoeficients(
      const ProblemData& p, const ProblemSolution& integer_solution,
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
  OPTSTAT SetUpAndRunBranchAndBound(const ProblemData& p,
                                    int num_nodes_limit,
                                    OPT_LP* lp,
                                    ProblemSolution* integer_solution);

  /// Returns a lower bound
  double BB(const ProblemData& p,
            int num_nodes_limit,
            int depth,
            double lower_bound,
            OPT_LP* lp,
            ProblemSolution* best_solution,
            vector<vector<short> >* fixed_vars,
            int* num_columns,
            double* pct_tree_solved,
            int* num_visited_nodes,
            OPTSTAT* status);

  // Funções para fixar o valor de uma variável de alocação (Xij) e continuar
  // o branch and bound. Elas armazenam o estado, fazem as modificações
  // necessárias nas colunas e chamam o BB. Depois que o BB retorna, elas
  // retornam as colunas e a matriz 'fixed_vars' ao seu estado original.
  enum FixingSense {
    FIX_ON_ZERO = 0,
    FIX_ON_ONE
  };

  void SetAndStoreFixedVariables(
      FixingSense sense, const ProblemData& p, int fixed_machine, int fixed_task,
      vector<vector<short> >* fixed, vector<short>* before_fixing);

  bool ShouldRemoveColumnWhenFixing(
      FixingSense sense, const VariableGeracaoColunas& var,
      int fixed_machine, int fixed_task);

  double FixVariableAndContinueBB(
    FixingSense fixing_sense, int fixed_machine, int fixed_task,
    int num_nodes_limit, double lower_bound, const ProblemData& p,
    OPT_LP* lp, ProblemSolution* best_solution,
    vector<vector<short> >* fixed_vars,
    int* num_columns, int depth, double* pct_tree_solved,
    int* num_visited_nodes, OPTSTAT* status);

  // Usa uma função heurística para selecionar variáveis candidatas
  // a serem fixadas. Depois pega essas candidatas e submete a um
  // rápido ("raso") BB para escolher a que gera o melhor lower bound.
  int SelectFixedVariable(const ProblemData& p, int num_vars_lookup,
                          const vector<vector<double> >& x, OPT_LP* lp,
                          ProblemSolution* best_solution,
                          vector<vector<short> >* fixed_vars, int* num_columns,
                          int* fixed_machine, int* fixed_task);

  double EvaluateVariableToFix(double binary_var);

  OPT_LP* lp_;
  int num_pivo_;
  int column_count_;

  /** Hash which associates the column number with the Variable object. */
  VariableGeracaoColunasHash vHash_;

  /** Hash which associates the row number with the Constraint object. */
  ConstraintGeracaoColunasHash cHash_;

  static const int kMaxNumberColumns_ = 40000;
  static const int kNumColumnsToRemove_ = 10000;
};
