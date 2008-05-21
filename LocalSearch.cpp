#include "LocalSearch.h"

#using <System.dll>
using namespace System::Diagnostics;

#include "ProblemSolution.h"
#include "ConstructiveHeuristics.h"
#include "ProblemData.h"
#include "SolverFormulacaoPadrao.h"

void LocalSearch::SimpleOPTSearch(const ProblemSolution& init_sol,
								  int max_opt,
								  int k_step,
								  int total_time,
								  int initial_time,
								  int ls_time,
								  ProblemSolution* sol) {
    const double kInfinity = 1e9;
	double UB = kInfinity;
	// to keep the time
    Stopwatch^ sw = gcnew Stopwatch();

    // creates solver
    SolverFormulacaoPadrao solver(Globals::instance());
    solver.Init();
    *sol = init_sol;

	// Tries to generate the first solution
	int status;
	sw->Start();
	status = solver.SolveTLAndUB(initial_time, UB);
	sw->Stop();
	if (status == OPTSTAT_NOINTEGER || status == OPTSTAT_INFEASIBLE)
		return; // error, could not generate first solution

	int cons = -1;
    int opt = k_step;
    do {
        ProblemSolution oldsol = *sol;
        if (cons >= 0) solver.RemoveConstraint(cons);
        cons = solver.AddConsMaxAssignmentChanges(*sol, opt);
        status = solver.Solve(ls_time);
        solver.GenerateSolution(sol);
        cout << "OPT = " << opt << " - new solution: " << sol->cost() << endl;
        if (*sol == oldsol) {
            opt += k_step;
            cout << "No change, moving to OPT = " << opt << endl;
        } else {
            opt = k_step;
        }
    } while (opt < max_opt);
}

uint64 LocalSearch::EllipsoidalSearch(const ProblemSolution& x1,
									  const ProblemSolution& x2,
									  int max_opt,
									  int k_step,
									  uint64 step_time,
									  uint64 total_time,
									  ProblemSolution* final_sol) {
	double kInfinity = 1e9;

	// to keep the time
	uint64 elapsed_time = 0;
    Stopwatch^ sw = gcnew Stopwatch();

    // creates solver
    SolverFormulacaoPadrao solver(Globals::instance());
    solver.Init();

	ProblemSolution best_sol(Globals::instance());
	best_sol = x1;

    int opt = 0;
	for (int opt = k_step; opt <= max_opt && elapsed_time < total_time; opt += k_step) {
		solver.UpdateConsMaxAssignmentChangesEllipsoidal(x1, x2, opt);
		
		sw->Start();
        int status = solver.SolveTLAndUB(step_time, kInfinity);
		sw->Stop();
		elapsed_time += sw->ElapsedMilliseconds;
		sw->Reset();

		// if valid solution found with better cost
		if (status == OPTSTAT_MIPOPTIMAL || status == OPTSTAT_FEASIBLE) {
			solver.GenerateSolution(final_sol);
			if (final_sol->cost() < best_sol.cost())
				best_sol = *final_sol;
		}
	}
	*final_sol = best_sol;
	return elapsed_time;
}

void LocalSearch::MultiEllipsoidalSearch() {
	vector<ProblemSolution> pool(5, ProblemSolution(Globals::instance()));
	vector<ProblemSolution> opt_pool(30, ProblemSolution(Globals::instance()));
	for (int i = 0; i < static_cast<int>(pool.size()); i++) {
		ConstructiveHeuristics::RandomStupid(*Globals::instance(), &pool[i]);
		LocalSearch::SimpleOPTSearch(pool[i], 2, 1, 30, 5, 5, &opt_pool[i]);
	}
	
	int max_dist = 0;
	int sol_i, sol_j;
	for (int i = 0; i < static_cast<int>(pool.size()); ++i) {
		for (int j = i + 1; j < static_cast<int>(pool.size()); ++j) {
			int d = pool[i].Distance(pool[j]);
			cout << "Distance between " << i << " and " << j << " is " << d << endl;
			if (d > max_dist) {
				max_dist = d;
				sol_i = i;
				sol_j = j;
			}
		}
	}
	cout << "Most distant solutions have distance = " << max_dist << endl;
	ProblemSolution final_sol(Globals::instance());
	LocalSearch::EllipsoidalSearch(pool[sol_i], pool[sol_j], 20, 1, 100, 10000, &final_sol);
}

uint64 LocalSearch::VNSBra(uint64 total_time_limit,
						   uint64 node_time_limit,
						   int k_step,
						   ProblemSolution* final_sol) {
    const double kInfinity = 1e9;
    int iter = 0;

    // to keep the time
    uint64 elapsed_time = 0;
    Stopwatch^ sw = gcnew Stopwatch();

    ProblemSolution x_ini(Globals::instance()), x_opt(Globals::instance());
    SolverFormulacaoPadrao solver_intensification(Globals::instance());
	SolverFormulacaoPadrao solver_diversification(Globals::instance());
    solver_intensification.Init();
	solver_diversification.Init();

    int k_cur = k_step;
    uint64 TL = total_time_limit;
    double UB = kInfinity;

    solver_intensification.SolveTLAndUB(TL, UB, true); // status = MIPSOLVE(TL, UB, first = true, x_opt, f_opt)
    solver_intensification.GenerateSolution(&x_opt);// creates initial solution

    ProblemSolution x_cur = x_opt;
    double f_cur = x_opt.cost();
    while (elapsed_time < total_time_limit) {
        cout << "Iteration " << ++iter << " - elapsed time: " << elapsed_time / 1000 << "s" << endl;
		elapsed_time += VNSIntensification(
			&solver_intensification,
			Globals::instance()->n(),
			1,
			total_time_limit - elapsed_time,
			node_time_limit,
			&x_cur);

        if (x_cur.cost() < x_opt.cost()) {  //f_cur < f_opt
            x_opt = x_cur;  // and f_opt = f_cur;
            k_cur = k_step;
        } else {
            k_cur = k_cur + k_step;
        }

        // stopping condition
        if (x_opt.cost() == Globals::instance()->optimal()) {
            break;
        }

        bool cont = true;
        while (cont && elapsed_time < total_time_limit && k_cur + k_step <= Globals::instance()->n()) {
            int last_cons_less = solver_diversification.AddConsMinAssignmentChanges(x_opt, k_cur);  // add constraint k_cur <= delta(x, x_opt)
            int last_cons_greater = solver_diversification.AddConsMaxAssignmentChanges(x_opt, k_cur + k_step);  // add constraint delta(x, x_opt) <= k_cur + k_step
            TL = total_time_limit - elapsed_time;
            UB = kInfinity; //UB = x_opt.cost();

            sw->Start();
            // first = true implies first best strategy, first = false implies best overall strategy
            int status = solver_diversification.SolveTLAndUB(TL / 1000, UB, true); // status = MIPSOLVE(TL, UB, first = true/false, x_cur, f_cur)
            sw->Stop();
            elapsed_time += sw->ElapsedMilliseconds;
            sw->Reset();
            cout << "Shaking Step - RHS = [" << k_cur << "," << k_cur + k_step << "], TL = " << TL / 1000 << "s, UB = " << UB << ", status = " << status << endl;

            solver_diversification.GenerateSolution(&x_cur);
            solver_diversification.RemoveConstraint(last_cons_less, last_cons_greater);  // remove last two added constraints
            cont = false;
            if (status == OPTSTAT_OPTIMALINFEAS || status == OPTSTAT_INFEASIBLE) {
                cont = true;
                k_cur = k_cur + k_step;
                cout << "Shaking Step - infeasible, continuing with disc size " << k_cur << endl;
            }
        }
    }
	*final_sol = x_opt;
	return elapsed_time;
}

uint64 LocalSearch::VNSIntensification(SolverFormulacaoPadrao *solver_intensification,
									   int max_opt,
									   int k_step,
									   uint64 total_time_limit,
									   uint64 node_time_limit,
									   ProblemSolution* x_cur) {
    // to keep the time
	uint64 elapsed_time = 0;
    Stopwatch^ sw = gcnew Stopwatch();

	// constraints added during search and that will be removed after it
	deque<int> added_cons;

    bool cont = true;
    int rhs = k_step;
    while (cont && elapsed_time < total_time_limit && rhs < max_opt) {
        uint64 TL = min(node_time_limit, total_time_limit - elapsed_time);
        added_cons.push_back(solver_intensification->AddConsMaxAssignmentChanges(*x_cur, rhs));  // add local branching constraint delta(x, x_cur) <= rhs
        double UB = x_cur->cost();  // UB = f_cur;

		//solver_intensification->AddConsIntegerSolutionStrongCuttingPlane(*x_cur);

        // runs CPLEX
        sw->Start();
        int status = solver_intensification->SolveTLAndUB(TL / 1000, UB);  // status = MIPSOLVE(TL, UB, first = false, x_next, f_next)
        sw->Stop();
        elapsed_time += sw->ElapsedMilliseconds;
        sw->Reset();

        cout << "VND search - RHS = " << rhs << ", TL = " << TL / 1000 << "s, UB = " << UB << ", status = " << status << endl;
        switch(status) {
            case OPTSTAT_MIPOPTIMAL:
                cout << "VND search - optimal, reverting constraint to delta(x, x_cur) >= " << rhs + k_step << endl;
                // x_cur = x_next, f_cur = f_next;
				solver_intensification->GenerateSolution(x_cur);
				// reverse last local branching constraint into delta(x, x_cur) >= rhs + 1
                solver_intensification->ReverseConstraint(added_cons.back(), rhs + k_step);
                rhs = k_step;
                break;
            case OPTSTAT_FEASIBLE:
                cout << "VND search - feasible, reverting constraint to delta(x, x_cur) >= " << k_step << endl;
				//x_cur = x_next, f_cur = f_next;
                solver_intensification->GenerateSolution(x_cur);
				// reverse last local branching constraint into delta(x, x_cur) >= k_step
                solver_intensification->ReverseConstraint(added_cons.back(), k_step);
                rhs = k_step;
                break;
            case OPTSTAT_INFEASIBLE:
                cout << "VND search - proven infeasible, removing last constraint, new RHS = " << rhs + k_step << endl;
				// remove last local branching constraint;
                solver_intensification->RemoveConstraint(added_cons.back());
                added_cons.pop_back();
                rhs += k_step;
                break;
            case OPTSTAT_NOINTEGER:
                cout << "VND search - infeasible, breaking out of the search" << endl;
                cont = false;
                break;
        }
        // stopping condition
        if (rhs >= Globals::instance()->n() ||
			((status == OPTSTAT_MIPOPTIMAL || status == OPTSTAT_FEASIBLE) && x_cur->cost() == Globals::instance()->optimal())) {
			break;
        }
    }
	
	// from the back so the constraint re-numbering doesn't screw up the removal
	for (int i = added_cons.size() - 1; i >= 0; --i) {
		// remove all added constraints
		solver_intensification->RemoveConstraint(added_cons[i]);
	}
	return elapsed_time;
}

void LocalSearch::MIPTabuSearch(ProblemSolution* final_sol) {
	// 1. Generate a "clean" MIP model for the GAP
	// 2. Solve that model and generate curr_sol
	// 2. Initialize short term memory (TabuList = [ ])
	// 3. While (stopping criteria not met):  # stopping = time limit and number of iterations
	//   4. Use MIP_LocalSearch(upper_bound = best_solution) and take that as aspiration criteria
	//   5. Add TabuList restrictions to the MIP and solve it, generating new_sol
	//   6. Add the differences between the curr_sol and new_sol to the TabuList
}

uint64 LocalSearch::GenerateInitialSolution(SolverFormulacaoPadrao* solver, int total_time) {
	Stopwatch^ sw = gcnew Stopwatch();
	int init_sol_time = 1, status = -1;
	uint64 elapsed_time = 0;
	do {
		solver->Init();
		
		sw->Start();
		status = solver->SolveTLAndUB(init_sol_time, 1e9, true);
		sw->Stop();
		elapsed_time += sw->ElapsedMilliseconds;
		sw->Reset();

		init_sol_time *= 2;
	}  while (status != OPTSTAT_FEASIBLE && status != OPTSTAT_MIPOPTIMAL && elapsed_time < total_time);
	return elapsed_time;
}

uint64 LocalSearch::MIPMemetic(ProblemSolution* final_sol) {
	vector<int> params(Params::NUM_PARAMS);
	params[TOTAL_TIME] = 1000 * 60 * 60;
	params[NUM_ITERATIONS] = 10;
	params[POPULATION_SIZE] = 20;

	uint64 elapsed_time = 0;

	// Best solution overall
	ProblemSolution best_sol(Globals::instance());

	// Crossover (ellipsoidal) Params
	vector<int>  ellip_params(Params::NUM_PARAMS);
	ellip_params[MAX_OPT] = 20;
	ellip_params[STEP_OPT] = 5;
	ellip_params[STEP_TIME] = 10 * 1000;
	ellip_params[TOTAL_TIME] = 60 * 1000;

	// Intensification Params
	vector<int> inten_params(Params::NUM_PARAMS);
	inten_params[MAX_OPT] = Globals::instance()->n();
	inten_params[STEP_OPT] = 1;
	inten_params[STEP_TIME] = 20 * 1000;
	inten_params[TOTAL_TIME] = 80 * 1000;

	vector<ProblemSolution>* pop = new vector<ProblemSolution>(
		params[POPULATION_SIZE],
		ProblemSolution(Globals::instance()));
	vector<ProblemSolution>* new_pop = new vector<ProblemSolution>(
		params[POPULATION_SIZE],
		ProblemSolution(Globals::instance()));

	SolverFormulacaoPadrao solver_inten(Globals::instance());

	// for every element in the population, generates the first solution
	elapsed_time += GenerateInitialSolution(&solver_inten, params[TOTAL_TIME]);
	solver_inten.GenerateSolution(&best_sol);
	for (int i = 0;
		i < params[POPULATION_SIZE] && elapsed_time < params[TOTAL_TIME];
		++i) {
		// Generates first solutions
		elapsed_time += GenerateInitialSolution(&solver_inten, params[TOTAL_TIME]);
		solver_inten.GenerateSolution(&pop->at(i));
		if (pop->at(i).cost() < best_sol.cost())
			best_sol = pop->at(i);
		if (best_sol.cost() == Globals::instance()->optimal())
			goto finalize;
	}

	for (int iter = 0;
		iter < params[NUM_ITERATIONS] && elapsed_time < params[TOTAL_TIME];
		++iter) {

		vector<vector<bool> > used(
			params[POPULATION_SIZE],
			vector<bool>(params[POPULATION_SIZE], false));
		// randomly selects params[POPULATION_SIZE] pairs of solutions
		for (int sol = 0;
			sol < params[POPULATION_SIZE] && elapsed_time < params[TOTAL_TIME];
			++sol) {
			int parents[2];
			do {
				parents[0] = Globals::rg()->IRandom(0, params[POPULATION_SIZE] - 1);
				parents[1] = Globals::rg()->IRandom(0, params[POPULATION_SIZE] - 1);
			} while (used[min(parents[0], parents[1])][max(parents[0], parents[1])]);
			used[min(parents[0], parents[1])][max(parents[0], parents[1])] = true;
			// the crossover of the parents is done through the EllipsoidalSearch
			elapsed_time += EllipsoidalSearch(
				pop->at(parents[0]),
				pop->at(parents[1]),
				ellip_params[MAX_OPT],
				ellip_params[STEP_OPT],
				ellip_params[STEP_TIME],
				ellip_params[TOTAL_TIME],
				&new_pop->at(sol));
		}

		// intensifies each solution
		for (int i = 0;
			i < params[POPULATION_SIZE] && elapsed_time < params[TOTAL_TIME];
			++i) {
			elapsed_time += VNSIntensification(
				&solver_inten,
				inten_params[MAX_OPT],
				inten_params[STEP_OPT],
				inten_params[TOTAL_TIME],
				inten_params[STEP_TIME],
				&new_pop->at(i));
			if (new_pop->at(i).cost() < best_sol.cost()) {
				best_sol = new_pop->at(i);
				if (best_sol.cost() == Globals::instance()->optimal())
					goto finalize;
			}
		}

		// population = new population
		swap(pop, new_pop);
	}

finalize:
	*final_sol = best_sol;
	return elapsed_time;
}