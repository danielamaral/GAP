#pragma once

#include <fstream>
#include <sstream>
#include <vector>

#include <vcclr.h>
#using <System.dll>
using namespace System::Diagnostics;

#include "solver.h"
#include "opt_lp.h"
#include "ConstraintGeracaoColunas.h"
#include "Globals.h"
#include "ProblemSolution.h"
#include "VariableGeracaoColunas.h"
#include "EllipsoidalCut.h"
#include "VnsCut.h"

using namespace std;

class SolverGeracaoColunas;
class SolverGeracaoColunasFactory;

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


class SolverGeracaoColunas : public VnsSolver {
 public:
  SolverGeracaoColunas(ProblemData* problem_data);
  ~SolverGeracaoColunas();

  /********************************************************************
  **                SOLVING AND RESULTS METHODS                      **
  *********************************************************************/
	int Solve(const SolverOptions& options, SolverStatus* status);
  
  /// Initializes the problem (creates variables and constraints)
  void Init(const SolverOptions& options);

  /// Generates a ProblemSolution from the X vector
  void GenerateSolution(const ProblemData& p,
                        const VariableGeracaoColunasContainer& vContainer,
                        const OPT_LP& lp,
                        ProblemSolution* sol) const;
  void GenerateSolution(ProblemSolution* sol);

  /********************************************************************
  **                 LOCAL SEARCH CONSTRAINTS                        **
  *********************************************************************/
  // Adds a constraint: delta(x, sol) <= num_changes
  virtual int AddConsMaxAssignmentChanges(const ProblemSolution& sol,
                                          int num_changes);
  
  // Adds a constraint: delta(x, sol) >= num_changes
  virtual int AddConsMinAssignmentChanges(const ProblemSolution& sol,
                                          int num_changes);
  
  // Removes the constraint <cons_row> or the range [cons_row_begin, cons_row_end]
  virtual void RemoveConstraint(int cons_row);
  virtual void RemoveConstraint(int cons_row_begin, int cons_row_end);
  
  // Changes the sense of the constraint <cons_row> and updates the RHS side to <rhs>
  virtual void ReverseConstraint(int cons_row, int rhs);

  virtual int AddEllipsoidalConstraint(const ProblemSolution& x1,
                                       const ProblemSolution& x2,
                                       int k);

private:
  /********************************************************************
  **                       UTILITY METHODS                           **
  *********************************************************************/
  /// If the time to solve the problem has expired.
  bool TimeExpired() const;

  /// Generates a Xij matrix from the given fractionary solution
  void GetXijMatrix(const vector<double>& current_sol,
                    const VariableGeracaoColunasContainer& vContainer,
                    vector<vector<double> >* x) const;

  /// Utility functions to deal with the mapping of column indices to task lists.
  void AddTasksToColumnMap(int column_index, int num_tasks, short* tasks);
  const vector<short>& GetColumnTasks(int column_index) const;
  void RemoveColumnTasksFromMap(int column_index);

  // Function that correctly sets the final status based on the previous and the found.
  void SetStatus(const OPTSTAT& previous, const OPTSTAT& current, OPTSTAT* final);

  /********************************************************************
  **             VARIABLE CREATION + STABILIZATION                   **
  *********************************************************************/

  void AddNewColumnWithTasks(double cost, double lower_bound, double upper_bound,
                             int machine, int num_tasks, short* tasks,
                             VariableGeracaoColunasContainer* vContainer, OPT_LP* lp);

  void AddNewColumnReuseTasks(const VariableGeracaoColunas& var,
                              double lower_bound, double upper_bound,
                              VariableGeracaoColunasContainer* vContainer, OPT_LP* lp);

  void AddStabilizationColumnsWithCoeficients(
      const ProblemData& p, const vector<double>& relaxed_dual_values,
      VariableGeracaoColunasContainer* vContainer, OPT_LP* lp);

  OPTSTAT AddInitialColumns(bool first, int time_limit,
                            ProblemSolution* integer_solution);

  void AddIntegerSolutionColumnsWithCoeficients(
      const ProblemData& p, const ProblemSolution& integer_solution,
      VariableGeracaoColunasContainer* vContainer, OPT_LP* lp);

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
  void AdjustStabilizationBounds(
    const ProblemData& p, double bound,
    const VariableGeracaoColunasContainer& vContainer, OPT_LP* lp);

  void UpdateStabilizationCosts(
    const ProblemData& p, const vector<double>& dual_values, 
    VariableGeracaoColunasContainer* vContainer, OPT_LP* lp);

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
                           VariableGeracaoColunasContainer* vContainer,
                           OPT_LP* lp);

  /********************************************************************
  **                    BRANCH AND BOUND METHODS                     **
  *********************************************************************/
  OPTSTAT SetUpAndRunBranchAndBound(const ProblemData& p,
                                    int num_nodes_limit,
                                    double cut_off_value,
                                    bool first_only,
                                    OPT_LP* lp,
                                    ProblemSolution* integer_solution);

  /// Returns a lower bound
  double BB(const ProblemData& p,
            int num_nodes_limit,
            int depth,
            double lower_bound,
            double cut_off_value,
            bool first_only,
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
  void SetAndStoreFixedVariables(
      FixingSense sense, const ProblemData& p, int fixed_machine, int fixed_task,
      vector<vector<short> >* fixed, vector<short>* before_fixing);

  bool ShouldRemoveColumnWhenFixing(
      FixingSense sense, const VariableGeracaoColunas& var, int column_number,
      int fixed_machine, int fixed_task);

  double FixVariableAndContinueBB(
    FixingSense fixing_sense, int fixed_machine, int fixed_task,
    int num_nodes_limit, double lower_bound, double cut_off_value,
    bool first_only, const ProblemData& p,
    OPT_LP* lp, ProblemSolution* best_solution,
    vector<vector<short> >* fixed_vars,
    int* num_columns, int depth, double* pct_tree_solved,
    int* num_visited_nodes, OPTSTAT* status);

  // Usa uma função heurística para selecionar variáveis candidatas
  // a serem fixadas. Depois pega essas candidatas e submete a um
  // rápido ("raso") BB para escolher a que gera o melhor lower bound.
  int SelectFixedVariable(const ProblemData& p, int num_vars_lookup,
                          const vector<vector<double> >& x, double cut_off, OPT_LP* lp,
                          ProblemSolution* best_solution,
                          vector<vector<short> >* fixed_vars, int* num_columns,
                          int* fixed_machine, int* fixed_task);

  double EvaluateVariableToFix(double binary_var);

  OPT_LP* lp_;
  int total_num_nodes_;
  int column_count_;
  bool stabilize_column_generation_;
  bool has_generated_initial_columns_;

  /** Some interesting statistics from the solving */
  double root_lower_bound_;
  int node_with_best_result_;

  /** Internal variables to control the time limits */
  gcroot<Stopwatch ^> stopwatch_;
  uint64 time_limit_in_milliseconds_;

  /** Hash which associates the column number with the Variable object. */
  VariableGeracaoColunasContainer vContainer_;
  map<int, vector<short> > column_map_;

  /** Hash which associates the row number with the Constraint object. */
  ConstraintGeracaoColunasHash cHash_;

  /** Structure that keeps the added VNS and Ellipsoidal constraints */
  vector<VnsCut> vns_cuts_;
  vector<EllipsoidalCut> ellipsoidal_cuts_;

  static const int kMaxNumberColumns_ = 100000;
  static const int kNumColumnsToRemove_ = 20000;
};

class SolverGeracaoColunasFactory : public SolverFactory {
 public:
  SolverGeracaoColunasFactory() {}
  ~SolverGeracaoColunasFactory() {}
  Solver* NewSolver(ProblemData* problem_data) {
    return new SolverGeracaoColunas(problem_data);
  }
  VnsSolver* NewVnsSolver(ProblemData* problem_data) {
    return new SolverGeracaoColunas(problem_data);
  }
};