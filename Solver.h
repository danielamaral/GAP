#pragma once

#include "Globals.h"
#include "logging.h"
#include "opt_row.h"
#include "ProblemSolution.h"

#include <sstream>

class ProblemData;
class ProblemSolution;

class Solver;
class VnsSolver;
class PopulateOptions;
class SolverStatus;
class SolverOptions;
class SolverFactory;

class SolverOptions {
public:
  virtual void Clear() {
    only_first_solution_ = false;
    cut_off_value_ = Globals::Infinity();
    use_stabilization_ = true;
    max_time_ = -1;  // No time limit.
    time_for_first_solution_ = 300;  // A standard initial value.
    rins_period_ = 10;
    emphasis_on_feasibility_ = true;
    emphasis_on_optimality_ = true;
    continue_from_previous_ = false;
  }

  SolverOptions() {
    Clear();
  }

  // Getters
  double cut_off_value() const { return cut_off_value_; }
  int max_time() const { return max_time_; }
  int time_for_first_solution() const { return time_for_first_solution_; }
  bool only_first_solution() const { return only_first_solution_; }
  bool use_stabilization() const { return use_stabilization_; }
  int rins_period() const { return rins_period_; }
  bool emphasis_on_feasibility() const { return emphasis_on_feasibility_; }
  bool emphasis_on_optimality() const { return emphasis_on_optimality_; }
  bool continue_from_previous() const { return continue_from_previous_; }

  // Setters
  void set_continue_from_previos(bool v) { continue_from_previous_ = v; }
  void set_emphasis_on_feasibility(bool v) { emphasis_on_feasibility_ = v; }
  void set_emphasis_on_optimality(bool v) { emphasis_on_optimality_ = v; }
  void set_cut_off_value(double v) { cut_off_value_ = v; }
  void set_max_time(int t) { max_time_ = t; }
  void set_only_first_solution(bool v) { only_first_solution_ = v; }
  void set_use_stabilization(bool v) { use_stabilization_ = v; }
  void set_time_for_first_solution(int t) { time_for_first_solution_ = (t > 0 ? t : 10); }
  void set_relative_time_for_first_solution(int instance_size) {
    time_for_first_solution_ = std::max(time_for_first_solution_,
                                        CalculateDefaultFirstSolutionTime(instance_size));
  }

 protected:
  int CalculateDefaultFirstSolutionTime(int instance_size) {
    int min_instance_size = 5 * 100;
    int time_for_min_size = std::min(60, max_time_);  // 1 min

    int max_instance_size = 80 * 1600;
    int time_for_max_size =  std::min(std::max(time_for_min_size, (int)(max_time_ * 0.2)),
                                      60 * 60 * 24);  // max 24h

    double f = (instance_size - min_instance_size) /
                static_cast<double>(max_instance_size - min_instance_size);

    return static_cast<int>(time_for_min_size +
                            f * (time_for_max_size - time_for_min_size));
  }

  bool only_first_solution_;
  double cut_off_value_;
  int max_time_;
  int time_for_first_solution_;
  bool use_stabilization_;
  int rins_period_;
  bool emphasis_on_feasibility_, emphasis_on_optimality_;
  bool continue_from_previous_;
};


class PopulateOptions : public SolverOptions {
public:
  virtual void Clear() {
    SolverOptions::Clear();
    num_max_solutions_ = 30;
    gap_ = 1e+75;
    replace_mode_ = 2;
    intensity_ = 3;
  }

  PopulateOptions() {
    Clear();
  }

  // Getters
  int num_max_solutions() const { return num_max_solutions_; }
  double gap() const { return gap_; }
  int replace_mode() const { return replace_mode_; }
  int intensity() const { return intensity_; }

  // Setters
  void set_num_max_solutions(int x) { num_max_solutions_ = x; }
  void set_gap(double x) { gap_ = x; }
  void set_replace_mode(int v) { CHECK(0 <= v && v <= 2); replace_mode_ = v; }
  void set_intensity(int v) { CHECK(0 <= v && v <= 4); intensity_ = v; }

 protected:
  int num_max_solutions_;
  double gap_;
  int replace_mode_;
  int intensity_;
};

class SolverStatus {
public:
	SolverStatus() : status(-1), str_status("infeasible"), final_sol(Globals::instance()),
		gap_relative(0.0), gap_absolute(0.0), time(0.0), time_to_best_solution(0.0),
    total_num_nodes(0), node_with_best_result(-1) { }

	string ToString() {
		stringstream ss;
		ss << final_sol.cost() << " ";
		if (str_status != "heuristic")
			ss << gap_relative << " " << gap_absolute << " ";
		ss << time << " " << time_to_best_solution << " " << str_status
       << " " << total_num_nodes << " " << node_with_best_result;
		return ss.str();
	}
	int status;
	string str_status;

	ProblemSolution final_sol;
	
	double gap_relative;
	double gap_absolute;
	double time;
  double time_to_best_solution;
  int total_num_nodes;
  int node_with_best_result;
};

class PopulateStatus : public SolverStatus{
public:
	PopulateStatus() { SolverStatus(); }
  double mean_objective_value;
  int num_replaced_sols;
  vector<ProblemSolution> solution_set;
};

class SolverFactory {
 public:
  SolverFactory() {}
  virtual ~SolverFactory() {};
  virtual Solver* NewSolver(ProblemData* problem_data) = 0;
  virtual VnsSolver* NewVnsSolver(ProblemData* problem_data) = 0;
};

/**
* Defines a generic solver.
*/
class Solver
{
public:
   /**
   * Default Constructor.
   * @param aProblemData The problem's input data.
   */
   Solver(ProblemData *aProblemData);

   /** Destructor. */
   ~Solver();

   /**
   * Solves the problem.
   * @return The solution status.
   */
   virtual int Solve(const SolverOptions& options,
                     SolverStatus* status) = 0;
   virtual int Populate(const PopulateOptions& options,
                        PopulateStatus* status) = 0;
   virtual void Init(const SolverOptions& options) = 0;

   /**
   * Processes the variable values and populates the output class.
   * @param ps A reference to the class to be populated.
   */
	virtual void GenerateSolution(ProblemSolution *ps) = 0;

protected:
   /** A reference to the problem's input data. */
   ProblemData* problem_data_;
};


class VnsSolver : public Solver {
 public:
  VnsSolver(ProblemData *aProblemData) : Solver(aProblemData) { }
  // Adds a constraint: delta(x, sol) <= num_changes
  virtual int AddConsMaxAssignmentChanges(const ProblemSolution& sol,
                                          int num_changes) = 0;
  
  // Adds a constraint: delta(x, sol) >= num_changes
  virtual int AddConsMinAssignmentChanges(const ProblemSolution& sol,
                                          int num_changes) = 0;
  
  // Removes the constraint <cons_row> or the range [cons_row_begin, cons_row_end]
  virtual void RemoveConstraint(int cons_row) = 0;
  virtual void RemoveConstraint(int cons_row_begin, int cons_row_end) = 0;
  
  // Changes the sense of the constraint <cons_row> and updates the RHS side to <rhs>
  virtual void ReverseConstraint(int cons_row, int rhs) = 0;

  // Adds an ellipsoidal constraint with solutions in X with 
  virtual int AddEllipsoidalConstraint(const vector<const ProblemSolution*>& x,
                                       OPT_ROW::ROWSENSE constraint_sense,
                                       int F) = 0;

  virtual int AddEllipsoidalConstraintMaxDistance(
      const vector<const ProblemSolution*>& x, int RHS) {
    return AddEllipsoidalConstraint(x, OPT_ROW::GREATER, RHS);
  }

  virtual int AddEllipsoidalConstraintMinDistance(
      const vector<const ProblemSolution*>& x, int RHS) {
    return AddEllipsoidalConstraint(x, OPT_ROW::LESS, RHS);
  }

  virtual void UpdateConsUpperBound(double upper) = 0;
};