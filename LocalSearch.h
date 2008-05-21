#ifndef LOCALSEARCH_H
#define LOCALSEARCH_H

#include <deque>
#include <hash_map>
#include <vector>
#include <sstream>

#include "Globals.h"

class ProblemSolution;
class SolverFormulacaoPadrao;

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

    void SimpleOPTSearch(const ProblemSolution& init_sol,
		int max_opt,
		int k_step,
		int total_time,
		int initial_time,
		int ls_time,
		ProblemSolution* sol);
	uint64 EllipsoidalSearch(
		const ProblemSolution& x1,
		const ProblemSolution& x2,
		int max_opt,
		int k_step,
		uint64 step_time,
		uint64 total_time,
		ProblemSolution* final_sol);
	void MultiEllipsoidalSearch();
	// VNS, returns the total elapsed time in the method
	uint64 VNSIntensification(
		SolverFormulacaoPadrao *solver_intensification,
		int max_opt,
		int k_step,
		uint64 total_time_limit,
		uint64 node_time_limit,
		ProblemSolution* x_cur);
    uint64 VNSBra(
		uint64 total_time_limit,
		uint64 node_time_limit,
		int k_step,
		ProblemSolution* sol);
	// Tabu
	void MIPTabuSearch(
		ProblemSolution* final_sol);
	// Memetic
	std::string PrintPopulation(const std::vector<ProblemSolution>& pop);
	uint64 GenerateInitialSolution(SolverFormulacaoPadrao* solver, int total_time);
	void RandomizeSolution(ProblemSolution* sol, int exchanges);
	uint64 MIPMemetic(
		ProblemSolution* final_sol);
};

#endif