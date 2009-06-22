#ifndef LOCALSEARCH_H
#define LOCALSEARCH_H

#include <deque>
#include <hash_map>
#include <vector>
#include <sstream>

#include "Globals.h"
#include "SolverFormulacaoPadrao.h"

class ProblemSolution;
class FixedSizeSolutionSet;

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

  void SimpleOPTSearch(VnsSolver* solver,
                       int opt,
                       uint64 time_limit,
                       int log,
                       ProblemSolution* sol);

  void GetEllipsoidalBounds(const vector<const ProblemSolution*>& sols,
                            int* minimum_f, int* maximum_f);

  bool EllipsoidalSearch(
      VnsSolver* solver,
      const vector<const ProblemSolution*>& sols,
      uint64 total_time_ms,
      uint64 step_time_ms,
      int log,
      int num_solutions,
      uint64* time_elapsed_ms,
      vector<ProblemSolution*>* final_sols,
      double initial_UB = 1e18);

  void MultiEllipsoidalSearch();

	// VNS, returns the total elapsed time in the method
	uint64 VNSIntensification(
      VnsSolver *solver_intensification,
      int initial_opt,
      int step_opt,
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
  void RandomlyMutateSolution(int num_mutations, ProblemSolution* sol);
  void PickNSolutions(int N, int log, const FixedSizeSolutionSet& R,
                      vector<const ProblemSolution*>* sols);
  void PathRelink(
    SolverFactory* solver_factory,
    uint64 total_time_ms,
    uint64 local_search_time_ms,
    int log_level,
    SolverStatus* status);

  void CalculateDistances(int num_sols, const FixedSizeSolutionSet& R,
                          const vector<vector<int> >& distance,
                          vector<pair<int, vector<int> > >* sols);

  void PostProcessing(
      SolverFactory* solver_factory,
      uint64 total_time_ms,
      uint64 local_search_time_ms,
      int log,
      SolverStatus* final_status);
};

#endif
