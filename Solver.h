#pragma once

#include "Globals.h"
#include "ProblemSolution.h"

#include <sstream>

class ProblemData;
class ProblemSolution;

class Solver;
class VnsSolver;
class SolverStatus;
class SolverOptions;
class SolverFactory;

class SolverOptions {
public:
  void Clear() {
    only_first_solution_ = false;
    cut_off_value_ = Globals::Infinity();
    use_stabilization_ = true;
    max_time_ = -1;  // No time limit.
    time_for_first_solution_ = 10;  // A standard initial value.
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

  // Setters
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
};

class SolverStatus {
public:
	SolverStatus() : status(-1), final_sol(Globals::instance()),
		gap_relative(0.0), gap_absolute(0.0), time(0.0), total_num_nodes(0),
    node_with_best_result(-1) { }

	string ToString() {
		stringstream ss;
		ss << final_sol.cost() << " ";
		if (str_status != "heuristic")
			ss << gap_relative << " " << gap_absolute << " ";
		ss << time << " " << str_status << " " << total_num_nodes
       << " " << node_with_best_result;
		return ss.str();
	}
	int status;
	string str_status;

	ProblemSolution final_sol;
	
	double gap_relative;
	double gap_absolute;
	double time;
  int total_num_nodes;
  int node_with_best_result;
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

  // Adds an ellipsoidal constraint with solutions <x1> and <x2>
  virtual int AddEllipsoidalConstraint(const ProblemSolution& x1,
                                       const ProblemSolution& x2,
                                       int k) = 0;
};