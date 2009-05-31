#ifndef LOCALSEARCH_H
#define LOCALSEARCH_H

#include <deque>
#include <hash_map>
#include <vector>
#include <sstream>

#include "Globals.h"
#include "SolverFormulacaoPadrao.h"

class ProblemSolution;

namespace LocalSearch {
	enum Params {
		// Cplex params
		// Generic search params
		MAX_OPT = 0,
		STEP_OPT,
		STEP_TIME,
		TOTAL_TIME,
		NUM_ITERATIONS,
		// Memetic params
		INITIAL_OPT,
		POPULATION_SIZE,
		RANDOMIZE_STEPS,
		NUM_PARAMS
	};

  void SimpleOPTSearch(
      const ProblemSolution& init_sol,
      int max_opt,
      int k_step,
      int total_time,
      int initial_time,
      int ls_time,
      ProblemSolution* sol);

  void GetEllipsoidalBounds(const vector<const ProblemSolution*>& sols,
                            int* minimum_f, int* maximum_f);

  bool EllipsoidalSearch(
      VnsSolver* solver,
      const vector<const ProblemSolution*>& sols,
      uint64 total_time_ms,
      uint64 step_time_ms,
      int log,
      bool only_first_solution,
      uint64* time_elapsed_ms,
      ProblemSolution* final_sol);

  void MultiEllipsoidalSearch();

	// VNS, returns the total elapsed time in the method
	uint64 VNSIntensification(
      VnsSolver *solver_intensification,
      int max_opt,
      uint64 total_time_ms,
      uint64 node_time_ms,
      int log_level,
      uint64* time_to_sol,
      ProblemSolution* x_cur);

  void VNSBra(
      SolverFactory* solver_factory,
      uint64 total_time_ms,
      uint64 node_time_ms,
      int k_step,
      SolverStatus* status);

	// Tabu
	void MIPTabuSearch(
		ProblemSolution* sol);
	// Memetic
	std::string PrintPopulation(const std::vector<ProblemSolution>& pop);
	uint64 GenerateInitialSolution(SolverFormulacaoPadrao* solver, int total_time);
	void RandomizeSolution(ProblemSolution* sol, int exchanges);
	void MIPMemetic(
		SolverStatus* final_status);

  // Path relink
  void RandomlyMutateSolutionWithPrecalc(const vector<pair<int, int> >& exchanges, 
                                         int num_mutations, ProblemSolution* sol);
  void RandomlyMutateSolution(int num_mutations, ProblemSolution* sol);
  void PathRelink(
    SolverFactory* solver_factory,
    uint64 total_time_ms,
    uint64 local_search_time_ms,
    int log_level,
    SolverStatus* status);
};

#endif
