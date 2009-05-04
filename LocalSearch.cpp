#include "LocalSearch.h"

#using <System.dll>
using namespace System::Diagnostics;

#include "ConstructiveHeuristics.h"
#include "logging.h"
#include "ProblemSolution.h"
#include "ProblemData.h"

void LocalSearch::SimpleOPTSearch(const ProblemSolution& init_sol,
                                  int max_opt,
                                  int k_step,
                                  int total_time,
                                  int initial_time,
                                  int ls_time,
                                  ProblemSolution* sol) {
	double UB = Globals::Infinity();
	// to keep the time
  Stopwatch^ sw = gcnew Stopwatch();

  // creates solver
  SolverFormulacaoPadrao solver(Globals::instance());
  solver.Init(SolverOptions());
  *sol = init_sol;

	// Tries to generate the first solution
  SolverOptions options;
  options.set_cut_off_value(UB);
  options.set_max_time(initial_time);
  SolverStatus dummy_status;

	sw->Start();
	int status = solver.Solve(options, &dummy_status);
	sw->Stop();
	if (status == OPTSTAT_NOINTEGER || status == OPTSTAT_INFEASIBLE)
		return; // error, could not generate first solution

	int cons = -1;
    int opt = k_step;
    do {
        ProblemSolution oldsol = *sol;
        if (cons >= 0) solver.RemoveConstraint(cons);
        cons = solver.AddConsMaxAssignmentChanges(*sol, opt);
        SolverOptions options; options.set_max_time(ls_time);
        status = solver.Solve(options, NULL);
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
                                      int step_time,
                                      int total_time,
                                      ProblemSolution* final_sol) {

	// to keep the time
	uint64 elapsed_time = 0;
  Stopwatch^ sw = gcnew Stopwatch();

  // creates solver
  SolverOptions options; options.set_max_time(step_time);
  SolverFormulacaoPadrao solver(Globals::instance());
  solver.Init(options);

	ProblemSolution best_sol(Globals::instance());
	best_sol = x1;

  int opt = 0;
	for (int opt = k_step; opt <= max_opt && elapsed_time < total_time; opt += k_step) {
		solver.UpdateConsMaxAssignmentChangesEllipsoidal(x1, x2, opt);
		
		sw->Start();
		int status = solver.Solve(options, NULL);
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

void LocalSearch::VNSBra(SolverFactory* solver_factory,
                         int total_time_limit,
                         int node_time_limit,
                         int k_step,
                         SolverStatus* status) {
  int iter = 0;

  // to keep the time
  uint64 elapsed_time = 0;
  Stopwatch^ sw = gcnew Stopwatch();

  SolverOptions initial_options;
  initial_options.set_only_first_solution(true);
  ProblemSolution x_ini(Globals::instance()), x_opt(Globals::instance());
  VnsSolver* solver_intensification = solver_factory->NewVnsSolver(Globals::instance());
  VnsSolver* solver_diversification = solver_factory->NewVnsSolver(Globals::instance());
  solver_intensification->Init(initial_options);
	solver_diversification->Init(initial_options);

  int k_cur = k_step;
  int TL = total_time_limit;
	double UB = Globals::Infinity();

  SolverOptions options;
  options.set_cut_off_value(UB);
  options.set_max_time(TL);
  options.set_only_first_solution(true);
  // status = MIPSOLVE(TL, UB, first = true, x_opt, f_opt)
  solver_intensification->Solve(options, status);
  if (status->status == OPTSTAT_MIPOPTIMAL)
    return;
  solver_intensification->GenerateSolution(&x_opt);  // creates initial solution

  ProblemSolution x_cur(x_opt);
  double f_cur = x_opt.cost();
  while (elapsed_time < total_time_limit) {
    LOG(INFO) << "Iteration " << ++iter << " - elapsed time: "
              << static_cast<double>(elapsed_time) / 1000.0 << "s" << endl;
		elapsed_time += VNSIntensification(solver_intensification,
                                       Globals::instance()->n(),
                                       1,
                                       total_time_limit - static_cast<int>(elapsed_time),
                                       node_time_limit,
                                       &x_cur);

    if (x_cur.cost() < x_opt.cost()) {  //f_cur < f_opt
      x_opt = x_cur;  // and f_opt = f_cur;
      k_cur = k_step;
      LOG(INFO) << "VNSBra: found best solution, cost: " << x_opt.cost();
    } else {
      k_cur = k_cur + k_step;
      VLOG(1) << "VNSBra: increasing step + " << k_step << " = " << k_cur;
    }

    // stopping condition
    if (x_opt.cost() == Globals::instance()->optimal()) {
      break;
    }

    bool cont = true;
    while (cont &&
           elapsed_time < total_time_limit &&
           k_cur + k_step <= Globals::instance()->n()) {
      // add constraint k_cur <= delta(x, x_opt)
      int last_cons_less =
        solver_diversification->AddConsMinAssignmentChanges(x_opt, k_cur);

      // add constraint delta(x, x_opt) <= k_cur + k_step
      int last_cons_greater =
        solver_diversification->AddConsMaxAssignmentChanges(x_opt, k_cur + k_step);

      TL = total_time_limit - static_cast<int>(elapsed_time);

      sw->Start();
      SolverOptions options;
      // UB = infinity
      options.set_max_time(TL / 1000.0);  // Converts milliseconds to seconds.
      options.set_only_first_solution(true);

      // status = MIPSOLVE(TL, UB, first = true/false, x_cur, f_cur)
      LOG(INFO) << "VNSBra: Shaking Step - RHS = [" << k_cur << "," << k_cur + k_step
                << "], TL = " << TL / 1000 << "s, UB = " << UB;
      SolverStatus status;
      solver_diversification->Solve(options, &status);
      sw->Stop();
      elapsed_time += sw->ElapsedMilliseconds;
      sw->Reset();
      LOG(INFO) << "VNSBra: Shaking Step - RHS = [" << k_cur << "," << k_cur + k_step
                << "], TL = " << TL / 1000 << "s, UB = " << UB << ", status = "
                << status.status;

      // remove last two added constraints
      solver_diversification->RemoveConstraint(last_cons_greater);
      solver_diversification->RemoveConstraint(last_cons_less);  

      cont = false;
			if (status.status == OPTSTAT_FEASIBLE || status.status == OPTSTAT_MIPOPTIMAL) {
				cont = false;
        //solver_diversification->GenerateSolution(&x_cur);
        x_cur = status.final_sol;
				LOG(INFO) << "VNSBra: Shaking Step - stopping, found feasible/optimal solution: "
                  << x_cur.cost();
			} else {
        cont = true;
        k_cur = k_cur + k_step;
        LOG(INFO) << "VNSBra: Shaking Step - infeasible, continuing with disc size " << k_cur;
			}
    }
  }
	status->final_sol = x_opt;
	status->str_status = "heuristic";
	status->time = elapsed_time / 1000.0;
  delete solver_intensification;
  delete solver_diversification;
}

uint64 LocalSearch::VNSIntensification(VnsSolver *solver_intensification,
                                       int max_opt,
                                       int k_step,
                                       int total_time_limit,
                                       int node_time_limit,
                                       ProblemSolution* x_cur) {
  // to keep the time
	uint64 elapsed_time = 0;
  Stopwatch^ sw = gcnew Stopwatch();

	// constraints added during search and that will be removed after it
	deque<int> added_cons;

  bool cont = true;
  int rhs = k_step;
  while (cont && elapsed_time < total_time_limit && rhs < max_opt) {
    int TL = min(node_time_limit, total_time_limit - static_cast<int>(elapsed_time));
    LOG(INFO) << "VNSInt: Adding local branching constraint delta(x, x_cur) "
              << "<= rhs = " << rhs;
    added_cons.push_back(solver_intensification->AddConsMaxAssignmentChanges(*x_cur, rhs));

    VLOG(1) << "VNSInt: UB = f_cur = " << x_cur->cost();
    double UB = x_cur->cost();

    // runs CPLEX
    sw->Start();
    // status = MIPSOLVE(TL, UB, first = false, x_next, f_next)
    SolverOptions options; options.set_max_time(TL / 1000); options.set_cut_off_value(UB);
    VLOG(2) << "VNSInt: Calling solver_intensification with time: "
            << options.max_time() << ", cutoff: " << options.cut_off_value();
    SolverStatus status;
    solver_intensification->Solve(options, &status);
    VLOG(2) << "VNSInt: status = MIPSOLVE(TL=" << options.max_time() << ", UB="
            << options.cut_off_value() << ", first=false, x_next, f_next) = "
            << status.status;
    sw->Stop();
    elapsed_time += sw->ElapsedMilliseconds;
    sw->Reset();

    LOG(INFO) << "VND search - RHS = " << rhs << ", TL = " << TL / 1000 << "s, UB = "
              << UB << ", status = " << status.status;
    switch(status.status) {
      case OPTSTAT_MIPOPTIMAL:
        LOG(INFO) << "VND search - optimal, reverting constraint to delta(x, x_cur) >= "
                  << rhs + k_step;
        // x_cur = x_next, f_cur = f_next;
        //solver_intensification->GenerateSolution(x_cur);
        *x_cur = status.final_sol;
        // reverse last local branching constraint into delta(x, x_cur) >= rhs + k_step
        solver_intensification->ReverseConstraint(added_cons.back(), rhs + k_step);
        rhs = k_step;
        break;
      case OPTSTAT_FEASIBLE:
        LOG(INFO) << "VND search - feasible, reverting constraint to delta(x, x_cur) >= "
                  << k_step;
        //x_cur = x_next, f_cur = f_next;
        //solver_intensification->GenerateSolution(x_cur);
        *x_cur = status.final_sol;
        // reverse last local branching constraint into delta(x, x_cur) >= k_step
        solver_intensification->ReverseConstraint(added_cons.back(), k_step);
        rhs = k_step;
        break;
      case OPTSTAT_INFEASIBLE:
        LOG(INFO) << "VND search - proven infeasible, removing last constraint, "
                  << "new RHS = " << rhs + k_step;
        // remove last local branching constraint;
        solver_intensification->RemoveConstraint(added_cons.back());
        added_cons.pop_back();
        rhs += k_step;
        break;
      case OPTSTAT_NOINTEGER:
        LOG(INFO) << "VND search - infeasible, breaking out of the search";
        cont = false;
        break;
    }
    // stopping condition
    if (rhs >= Globals::instance()->n() ||
        ((status.status == OPTSTAT_MIPOPTIMAL || status.status == OPTSTAT_FEASIBLE) &&
          x_cur->cost() == Globals::instance()->optimal())) {
      LOG(INFO) << "VNSInt: stopping condition. RHS >= N or optimal cost solution found.";
      cont = false;
    }
  }
	
	// from the back so the constraint re-numbering doesn't screw up the removal
  VLOG(2) << "VNSInt: removing all " << added_cons.size() << " added constraints.";
	for (int i = added_cons.size() - 1; i >= 0; --i) {
		// remove all added constraints
    VLOG(3) << "VNSInt: removing " << i << "th constraint: " << added_cons[i];
		solver_intensification->RemoveConstraint(added_cons[i]);
	}

  LOG(INFO) << "VNSInt: finished, returning time " << elapsed_time;
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

void LocalSearch::MIPMemetic(SolverStatus *final_status) {
	vector<int> params(Params::NUM_PARAMS);
	params[TOTAL_TIME] = 1000 * 60 * 60;
	params[NUM_ITERATIONS] = 10;
	params[POPULATION_SIZE] = 20;

	uint64 elapsed_time = 0;
	Stopwatch^ sw = gcnew Stopwatch();

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

	// Generates first solution
	int init_sol_time = 1, status = -1;
	do {
		solver_inten.Init(SolverOptions());

    SolverOptions options;
    options.set_max_time(init_sol_time);
    options.set_only_first_solution(true);
		sw->Start();
		status = solver_inten.Solve(options, NULL);
    sw->Stop();
    elapsed_time += sw->ElapsedMilliseconds;
    sw->Reset();

		init_sol_time *= 2;
	}  while (status != OPTSTAT_FEASIBLE && status != OPTSTAT_MIPOPTIMAL);
	solver_inten.GenerateSolution(&best_sol);

	// for every element in the population, generates the first solution
	for (int i = 0; i < params[POPULATION_SIZE]; ++i) {
		int status = -1;
		do {
			cout << "Generating solution #" << i << endl;
			//ConstructiveHeuristics::RandomStupid(*Globals::instance(), &pop->at(i));
      SolverOptions options; options.set_max_time(10);
			solver_inten.Init(options);

			sw->Start();
			status = solver_inten.Solve(options, NULL);
			sw->Stop();
			elapsed_time += sw->ElapsedMilliseconds;
			sw->Reset();
		} while (status != OPTSTAT_FEASIBLE && status != OPTSTAT_MIPOPTIMAL);
		solver_inten.GenerateSolution(&pop->at(i));

		cout << "Generating solution #" << i << " cost: " << pop->at(i).cost() << endl;
		if (pop->at(i).cost() <= best_sol.cost()) {
			best_sol = pop->at(i);
			if (best_sol.cost() == Globals::instance()->optimal())
				goto finalize;
		}
	}

	for (int iter = 0;
		iter < params[NUM_ITERATIONS] && elapsed_time < params[TOTAL_TIME];
		++iter) {

		vector<vector<bool> > used(
			params[POPULATION_SIZE],
			vector<bool>(params[POPULATION_SIZE], false));
		// randomly selects params[POPULATION_SIZE] pairs of solutions
		for (int sol = 0; sol < params[POPULATION_SIZE]; ++sol) {
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
		for (int i = 0; i < params[POPULATION_SIZE] && elapsed_time < params[TOTAL_TIME]; ++i) {
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
	final_status->final_sol = best_sol;
	final_status->str_status = "heuristic";
	final_status->time = elapsed_time / 1000.0;
}