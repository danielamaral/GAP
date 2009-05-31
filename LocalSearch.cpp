#include "LocalSearch.h"

#using <System.dll>
using namespace System::Diagnostics;

#include <set>

#include "ConstructiveHeuristics.h"
#include "logging.h"
#include "ProblemSolution.h"
#include "ProblemData.h"
#include "SolutionSet.h"

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

void LocalSearch::GetEllipsoidalBounds(const vector<const ProblemSolution*>& sols,
                                       int* minimum_f, int* maximum_f) {
  // On top is the smallest among the largest
  priority_queue<int, vector<int>, greater<int> > biggest_weights;
  // On top is the largest among the smallest
  priority_queue<int> smallest_weights;

  int N = Globals::instance()->n();
  for (int task = 0; task < N; ++task) {
    // counts the weight of every machine assignment
    map<int, int> machine_to_counts;
    for (int sol = 0; sol < sols.size(); ++sol) {
      machine_to_counts[sols[sol]->assignment(task)] += 1;
    }
    for (map<int, int>::iterator it = machine_to_counts.begin();
      it != machine_to_counts.end(); ++it) {
      int c = it->second;
      if (biggest_weights.size() < N || c > biggest_weights.top()) {
        biggest_weights.push(c);
        if (biggest_weights.size() > N)
          biggest_weights.pop();
      }
      if (smallest_weights.size() < N || c < smallest_weights.top()) {
        smallest_weights.push(c);
        if (smallest_weights.size() > N)
          smallest_weights.pop();
      }
    }
    // Adds the zeroes wrt to the machines that weren't there
    for (int mac = 0; smallest_weights.top() > 0 &&
                      mac < (Globals::instance()->m() - machine_to_counts.size());
         ++mac) {
      smallest_weights.push(0);
      if (smallest_weights.size() > N)
        smallest_weights.pop();
    }
  }
  int max_feasible_dist = 0, min_feasible_dist = 0;
  while (!biggest_weights.empty()) {
    max_feasible_dist += biggest_weights.top();
    biggest_weights.pop();
  }
  if (smallest_weights.top() == 0)
    min_feasible_dist = 0;
  else while (!smallest_weights.empty()) {
    min_feasible_dist += smallest_weights.top();
    smallest_weights.pop();
  }

  *maximum_f = N * sols.size() - min_feasible_dist;
  *minimum_f = N * sols.size() - max_feasible_dist;
}

bool LocalSearch::EllipsoidalSearch(VnsSolver* solver,
                                    const vector<const ProblemSolution*>& sols,
                                    int max_opt,
                                    uint64 total_time_ms,
                                    uint64 step_time_ms,
                                    int log,
                                    uint64* time_elapsed_ms,
                                    ProblemSolution* final_sol) {

#define RHS(F) (Globals::instance()->n() * sols.size() - (F))
  // to keep the time
	*time_elapsed_ms = 0;
  Stopwatch^ sw = gcnew Stopwatch();

  // First, calculates the minimum and maximum slack possible
  int minimum_opt, maximum_opt;
  GetEllipsoidalBounds(sols, &minimum_opt, &maximum_opt);
  max_opt = min<int>(max_opt, maximum_opt);

  bool feasible = false;
  bool cont = true;
  int rhs = minimum_opt;
  deque<int> added_cons;
  while (cont && *time_elapsed_ms < total_time_ms && rhs <= max_opt) {
    uint64 TL = (*time_elapsed_ms + step_time_ms > total_time_ms ?
                 total_time_ms - *time_elapsed_ms : step_time_ms);
    double UB = (feasible ? final_sol->cost() : Globals::Infinity());
    VLOG(log) << "EllipsoidalSearch: adding constraint <= " << rhs << ", UB: "
              << UB << ", time limit: " << TL / 1000.0;

    // Adds constraint
    added_cons.push_back(solver->AddEllipsoidalMaxChangesConstraint(sols, RHS(rhs)));

    // Runs the solver
    sw->Start();
    SolverOptions options; options.set_max_time(TL / 1000.0); options.set_cut_off_value(UB);
    SolverStatus status;
		solver->Solve(options, &status);
		sw->Stop();
		*time_elapsed_ms += sw->ElapsedMilliseconds;
		sw->Reset();

    switch (status.status) {
      case OPTSTAT_MIPOPTIMAL:
        feasible = true;
        *final_sol = status.final_sol;
        VLOG(log) << "EllipsoidalSearch: optimal for rhs: " << rhs << ", cost : "
                  << status.final_sol.cost() << ", time elapsed: " << *time_elapsed_ms
                  << ", best solution: " << final_sol->cost();
        solver->ReverseConstraint(added_cons.back(), RHS(rhs + 1));
        rhs = minimum_opt;
        break;
      case OPTSTAT_FEASIBLE:
        feasible = true;
        *final_sol = status.final_sol;
        VLOG(log) << "EllipsoidalSearch: optimal for rhs: " << rhs << ", cost : "
                  << status.final_sol.cost() << ", time elapsed: " << *time_elapsed_ms
                  << ", best solution: " << final_sol->cost();
        solver->ReverseConstraint(added_cons.back(), RHS(1));
        rhs = minimum_opt;
        break;
      case OPTSTAT_INFEASIBLE:
        VLOG(log) << "EllipsoidalSearch: infeasible for rhs: " << rhs << ", time elapsed: "
                  << *time_elapsed_ms << ", new rhs: " << rhs + 1;
        solver->RemoveConstraint(added_cons.back());
        added_cons.pop_back();
        rhs += 1;
        break;
      case OPTSTAT_NOINTEGER:
        VLOG(log) << "EllipsoidalSearch: no integer solution for rhs:  " << rhs
                  << ", breaking out of search.";
        cont = false;
        break;
    }
	}

  while (!added_cons.empty()) {
    VLOG(log + 1) << "EllipsoidalSearch: removing constraint: " << added_cons.back();
    solver->RemoveConstraint(added_cons.back());
    added_cons.pop_back();
  }

  VLOG(log) << "EllipsoidalSearch: finished, time spent: " << *time_elapsed_ms / 1000.0
            << ", best solution: " << final_sol->cost();

	return feasible;
#undef RHS
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
			LOG(INFO) << "Distance between " << i << " and " << j << " is " << d;
			if (d > max_dist) {
				max_dist = d;
				sol_i = i;
				sol_j = j;
			}
		}
	}
	LOG(INFO) << "Most distant solutions have distance = " << max_dist;
	ProblemSolution final_sol(Globals::instance());
	//LocalSearch::EllipsoidalSearch(pool[sol_i], pool[sol_j], 20, 1, 100, 10000, &final_sol);
}

void LocalSearch::VNSBra(SolverFactory* solver_factory,
                         uint64 total_time_ms,
                         uint64 node_time_ms,
                         int k_step,
                         SolverStatus* status) {
  int iter = 0;
  status->str_status = "heuristic";

  // to keep the time
  uint64 elapsed_ms = 0;
  Stopwatch^ sw = gcnew Stopwatch();

  SolverOptions initial_options;
  initial_options.set_only_first_solution(true);
  ProblemSolution x_ini(Globals::instance()), x_opt(Globals::instance());
  VnsSolver* solver_intensification = solver_factory->NewVnsSolver(Globals::instance());
  VnsSolver* solver_diversification = solver_factory->NewVnsSolver(Globals::instance());
  solver_intensification->Init(initial_options);
	solver_diversification->Init(initial_options);

  int k_cur = k_step;
  int TL = total_time_ms;
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
  while (elapsed_ms < total_time_ms) {
    LOG(INFO) << "Iteration " << ++iter << " - elapsed time: "
              << static_cast<double>(elapsed_ms) / 1000.0 << "s" << endl;
    uint64 time_to_best = 0;
		uint64 intensif_time =
        VNSIntensification(solver_intensification, Globals::instance()->n(),
                           total_time_ms - static_cast<int>(elapsed_ms),
                           node_time_ms, FLAGS_v, &time_to_best, &x_cur);

    if (x_cur.cost() < x_opt.cost()) {  //f_cur < f_opt
      x_opt = x_cur;  // and f_opt = f_cur;
      k_cur = k_step;
      LOG(INFO) << "VNSBra: found best solution, cost: " << x_opt.cost() << ", time : "
                << (elapsed_ms + time_to_best) / 1000.0;
      status->time_to_best_solution = (elapsed_ms + time_to_best) / 1000.0;
    } else {
      k_cur = k_cur + k_step;
      VLOG(1) << "VNSBra: increasing step + " << k_step << " = " << k_cur;
    }
    elapsed_ms += intensif_time;

    // stopping condition
    if (x_opt.cost() == Globals::instance()->optimal()) {
      LOG(INFO) << "Solution cost == optimal cost, breaking.";
      status->str_status = "optimal";
      break;
    }

    bool cont = true;
    while (cont &&
           elapsed_ms < total_time_ms &&
           k_cur + k_step <= Globals::instance()->n()) {
      // add constraint k_cur <= delta(x, x_opt)
      int last_cons_less =
        solver_diversification->AddConsMinAssignmentChanges(x_opt, k_cur);

      // add constraint delta(x, x_opt) <= k_cur + k_step
      int last_cons_greater =
        solver_diversification->AddConsMaxAssignmentChanges(x_opt, k_cur + k_step);

      TL = total_time_ms - static_cast<int>(elapsed_ms);

      sw->Start();
      SolverOptions options;
      // UB = infinity
      options.set_max_time(TL / 1000.0);  // Converts milliseconds to seconds.
      options.set_only_first_solution(true);

      // status = MIPSOLVE(TL, UB, first = true/false, x_cur, f_cur)
      LOG(INFO) << "VNSBra: Shaking Step - RHS = [" << k_cur << "," << k_cur + k_step
                << "], TL = " << TL / 1000 << "s, UB = " << UB;
      SolverStatus diver_status;
      solver_diversification->Solve(options, &diver_status);
      sw->Stop();
      elapsed_ms += sw->ElapsedMilliseconds;
      sw->Reset();
      LOG(INFO) << "VNSBra: Shaking Step - RHS = [" << k_cur << "," << k_cur + k_step
                << "], TL = " << TL / 1000 << "s, UB = " << UB << ", status = "
                << diver_status.status;

      // remove last two added constraints
      solver_diversification->RemoveConstraint(last_cons_greater);
      if (last_cons_greater != last_cons_less)
        solver_diversification->RemoveConstraint(last_cons_less);  

      cont = false;
			if (diver_status.status == OPTSTAT_FEASIBLE ||
          diver_status.status == OPTSTAT_MIPOPTIMAL) {
				cont = false;
        //solver_diversification->GenerateSolution(&x_cur);
        x_cur = diver_status.final_sol;
				LOG(INFO) << "VNSBra: Shaking Step - stopping, found feasible/optimal solution: "
                  << x_cur.cost();
			} else {
        cont = true;
        k_cur = k_cur + k_step;
        LOG(INFO) << "VNSBra: Shaking Step - infeasible, continuing with disc size " << k_cur;
			}
    }

    if (k_cur + k_step >= Globals::instance()->n()) {
      LOG(INFO) << "k_cur + k_step >= N, breaking out, optimal solution.";
      status->str_status = "optimal";
      break;
    }
  }
	status->final_sol = x_opt;
	status->time = elapsed_ms / 1000.0;
  status->gap_absolute = status->gap_relative = 0.0;
  delete solver_intensification;
  delete solver_diversification;
}

uint64 LocalSearch::VNSIntensification(VnsSolver *solver_intensification,
                                       int max_opt,
                                       uint64 total_time_ms,
                                       uint64 node_time_ms,
                                       int log,
                                       uint64* time_to_sol,
                                       ProblemSolution* x_cur) {
#define RHS(X) (Globals::instance()->n() - (X))
  // to keep the time
	uint64 elapsed_time = 0;
  Stopwatch^ sw = gcnew Stopwatch();

	// constraints added during search and that will be removed after it
	deque<int> added_cons;

  bool cont = true;
  int rhs = 1;
  while (cont && elapsed_time < total_time_ms && rhs < max_opt) {
    int TL = min(node_time_ms, total_time_ms - static_cast<int>(elapsed_time));
    VLOG(log + 1)
        << "VNSInt: Adding local branching constraint delta(x, x_cur) "
        << "<= rhs = " << rhs << ", UB = f_cur = " << x_cur->cost();
    added_cons.push_back(
        solver_intensification->AddConsMaxAssignmentChanges(*x_cur, RHS(rhs)));
    double UB = x_cur->cost();

    // runs CPLEX
    sw->Start();
    // status = MIPSOLVE(TL, UB, first = false, x_next, f_next)
    SolverOptions options; options.set_max_time(TL / 1000); options.set_cut_off_value(UB);
    VLOG(log)
        << "VNSInt: Calling solver_intensification with time: "
        << options.max_time() << ", cutoff: " << options.cut_off_value();
    SolverStatus status;
    solver_intensification->Solve(options, &status);
    VLOG(log)
        << "VNSInt: status = MIPSOLVE(TL=" << options.max_time() << ", UB="
        << options.cut_off_value() << ", first=false, x_next, f_next) = "
        << status.status;
    sw->Stop();
    elapsed_time += sw->ElapsedMilliseconds;
    sw->Reset();

    VLOG(log) << "VND search - RHS = " << rhs << ", TL = " << TL / 1000 << "s, UB = "
              << UB << ", status = " << status.status;
    switch(status.status) {
      case OPTSTAT_MIPOPTIMAL:
        VLOG(log) << "VND search - optimal, reverting constraint to delta(x, x_cur) >= "
                  << rhs + 1;
        // x_cur = x_next, f_cur = f_next;
        if (status.final_sol.cost() < x_cur->cost())
          *time_to_sol = elapsed_time;
        *x_cur = status.final_sol;
        // reverse last local branching constraint into delta(x, x_cur) >= rhs + 1
        solver_intensification->ReverseConstraint(added_cons.back(), RHS(rhs + 1));
        rhs = 1;
        break;
      case OPTSTAT_FEASIBLE:
        VLOG(log) << "VND search - feasible, reverting constraint to delta(x, x_cur) >= "
                  << 1;
        //x_cur = x_next, f_cur = f_next;
        if (status.final_sol.cost() < x_cur->cost())
          *time_to_sol = elapsed_time;
        *x_cur = status.final_sol;
        // reverse last local branching constraint into delta(x, x_cur) >= 1
        solver_intensification->ReverseConstraint(added_cons.back(), RHS(1));
        rhs = 1;
        break;
      case OPTSTAT_INFEASIBLE:
        VLOG(log) << "VND search - proven infeasible, removing last constraint, "
                  << "new RHS = " << rhs + 1;
        // remove last local branching constraint;
        solver_intensification->RemoveConstraint(added_cons.back());
        added_cons.pop_back();
        rhs += 1;
        break;
      case OPTSTAT_NOINTEGER:
        VLOG(log) << "VND search - infeasible, breaking out of the search";
        cont = false;
        break;
    }
    // stopping condition
    if (rhs >= Globals::instance()->n() ||
        ((status.status == OPTSTAT_MIPOPTIMAL || status.status == OPTSTAT_FEASIBLE) &&
          x_cur->cost() == Globals::instance()->optimal())) {
      VLOG(log) << "VNSInt: stopping condition. RHS >= N or optimal cost solution found.";
      cont = false;
    }
  }
	
	// from the back so the constraint re-numbering doesn't screw up the removal
  VLOG(log + 1) << "VNSInt: removing all " << added_cons.size() << " added constraints.";
	for (int i = added_cons.size() - 1; i >= 0; --i) {
		// remove all added constraints
    VLOG(log + 2) << "VNSInt: removing " << i << "th constraint: " << added_cons[i];
    if (i == added_cons.size() - 1 || added_cons[i] != added_cons[i + 1])
		  solver_intensification->RemoveConstraint(added_cons[i]);
	}

  VLOG(log) << "VNSInt: finished, returning time " << elapsed_time;
	return elapsed_time;
#undef RHS
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
			/*elapsed_time += EllipsoidalSearch(
				pop->at(parents[0]),
				pop->at(parents[1]),
				ellip_params[MAX_OPT],
				ellip_params[STEP_OPT],
				ellip_params[STEP_TIME],
				ellip_params[TOTAL_TIME],
				&new_pop->at(sol));*/
		}

		// intensifies each solution
		for (int i = 0; i < params[POPULATION_SIZE] && elapsed_time < params[TOTAL_TIME]; ++i) {
			elapsed_time += VNSIntensification(
				&solver_inten,
				inten_params[MAX_OPT],
				inten_params[TOTAL_TIME],
				inten_params[STEP_TIME],
        FLAGS_v,
        NULL,
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

void LocalSearch::RandomlyMutateSolutionWithPrecalc(
    const vector<pair<int, int> >& exchanges, int num_mutations, ProblemSolution* sol) {
  for (int mutations = 0; mutations < num_mutations; ++mutations) {
    // Picks a random exchange
    int e = Globals::rg()->IRandom(0, exchanges.size() - 1);
    sol->Exchange(exchanges[e].first, exchanges[e].second);
  }
}

void LocalSearch::RandomlyMutateSolution(
    int num_mutations, ProblemSolution* sol) {
  int mutations = 0;
  while (mutations < num_mutations) {
    int task[2];
    task[0] = Globals::rg()->IRandom(0, sol->n() - 1);
    task[1] = Globals::rg()->IRandom(0, sol->n() - 1);
    // Picks a random pair of tasks
    if (task[0] != task[1] && sol->IsValidExchange(task[0], task[1])) {
      sol->Exchange(task[0], task[1]);
      ++mutations;
    }
  }
}

// TODO(danielrocha): keep track of elapsed time.
void LocalSearch::PathRelink(SolverFactory* solver_factory,
                             uint64 total_time_ms,
                             uint64 local_search_time_ms,
                             uint64 local_search_node_time_ms,
                             int log,
                             SolverStatus* status) {
  // Gerar um conjunto de soluções aleatórias em R
  const int kSetSize = 4;
  FixedSizeSolutionSet R(kSetSize);
  {
    VnsSolver* solver = solver_factory->NewVnsSolver(Globals::instance());
    SolverOptions options;
    options.set_only_first_solution(true);
    SolverStatus status;
    solver->Init(options);
    solver->Solve(options, &status);

    // Creates a vector of possible task exchanges
    const ProblemSolution& sol = status.final_sol;
    vector<pair<int, int> > exchanges;
    exchanges.reserve(sol.num_tasks() / 10 * sol.num_tasks());
    for (int i = 0; i < sol.num_tasks(); ++i)
      for (int j = i + 1; j < sol.num_tasks(); ++j)
        if (sol.IsValidExchange(i, j))
          exchanges.push_back(make_pair(i, j));

    for (int sol = 0; sol < kSetSize; ++sol) {
      ProblemSolution first_solution = status.final_sol;
      RandomlyMutateSolutionWithPrecalc(exchanges, Globals::rg()->IRandom(10, 30),
                                        &first_solution);
      R.AddSolution(first_solution);
      VLOG(log) << "PathRelink: adding initial solution " << sol << ", cost: "
                << first_solution.cost();
    }

    delete solver;
  }

  // Fazer busca local em cada solução, substituir pelo ótimo local
  {
    VnsSolver* solver = solver_factory->NewVnsSolver(Globals::instance());
    solver->Init(SolverOptions());
    for (int i = 0; i < kSetSize; ++i) {
      ProblemSolution s = R.GetSolution(i);
      uint64 time_to_best = 0;
      VLOG(log) << "PathRelink: updating solution " << i << " with its local optima.";
      LocalSearch::VNSIntensification(solver, 6, local_search_time_ms,
                                      local_search_node_time_ms,
                                      log + 1, &time_to_best, &s);
      if (R.AddSolution(s)) {
        VLOG(log) << "PathRelink: updated solution " << i << " with new local minima: "
                  << s.cost();
      }
    }

    delete solver;
  }

  // Enquanto criterio de parada nao for atingido
  while (true) {
    // Um conjunto de soluções a serem otimizadas
    vector<ProblemSolution*> S;

    // Achar as X melhores soluções no caminho de A para B, adicionar a S
    const int X = 8;
    while (S.size() < X) {
      VLOG(log) << "PathRelink: relink step, " << X - S.size() << " still remaining.";
      // Escolher aleatóriamente duas soluções de R
      const int kNumSolEllipsoidal = 2;
      set<int> sol_indices;
      vector<const ProblemSolution*> sol_in_search;
      sol_in_search.reserve(kNumSolEllipsoidal);
      while (sol_indices.size() < kNumSolEllipsoidal) {
        int sol_to_insert = Globals::rg()->IRandom(0, R.size() - 1);
        if (sol_indices.insert(sol_to_insert).second) {
          sol_in_search.push_back(R.GetSolutionPtr(sol_to_insert));
          VLOG(log)
              << "Selecting for path relink solution " << sol_to_insert << ", cost: "
              << sol_in_search[sol_in_search.size() - 1]->cost();
        }
      }
      // TODO(danielrocha): talvez fazer mutacao nas solucoes?

      uint64 time_elapsed_ms = 0;
      ProblemSolution* new_solution = new ProblemSolution(Globals::instance());
      VnsSolver* solver_diver = solver_factory->NewVnsSolver(Globals::instance());
      solver_diver->Init(SolverOptions());
      bool feasible = 
          LocalSearch::EllipsoidalSearch(
              solver_diver, sol_in_search, Globals::instance()->n(),
              local_search_time_ms, local_search_node_time_ms,
              log, &time_elapsed_ms, new_solution);
      delete solver_diver;

      if (feasible) {
        S.push_back(new_solution);
        VLOG(log) << "PathRelink: inserted result of path relink into S, cost: "
                  << new_solution->cost() << ", num solutions: " << S.size();
      } else {
        delete new_solution;
        VLOG(log) << "PathRelink: path relink infeasible, S size: " << S.size();
      }
    }

    // Enquanto existirem soluções a ser otimizadas em S:
    while (S.size() > 0) {
      VnsSolver* solver_inten = solver_factory->NewVnsSolver(Globals::instance());
      solver_inten->Init(SolverOptions());

      // Escolher uma solução aleatória, otimizar, remover de S
      int picked_sol = Globals::rg()->IRandom(0, S.size() - 1);
      ProblemSolution* s = S[picked_sol];
      uint64 time_to_best = 0;
      VLOG(log) << "PathRelink: intensifying solution from S, current cost: " << s->cost()
                << ", position: " << picked_sol << ", S size: " << S.size();
      LocalSearch::VNSIntensification(solver_inten, 6, local_search_time_ms,
                                      local_search_node_time_ms,
                                      log + 1, &time_to_best, s);
      delete solver_inten;

      // Tentar adicionar o ótimo local a R
      if (R.AddSolution(*s)) {
        VLOG(log) << "PathRelink: added local optima to R, new cost: " << s->cost();
      } else {
        VLOG(log) << "PathRelink: DIDN'T add local optima to R, new cost: " << s->cost();
      }

      // Remove a solucao de S - transfere pro final e da um pop_back
      swap(S[picked_sol], S[S.size() - 1]);
      delete S[S.size() - 1];
      S.pop_back();
    }
  }
}