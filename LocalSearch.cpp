#include "LocalSearch.h"

#using <System.dll>
using namespace System::Diagnostics;

#include <set>

#include "ConstructiveHeuristics.h"
#include "logging.h"
#include "ProblemSolution.h"
#include "ProblemData.h"
#include "SolutionSet.h"

void LocalSearch::SimpleOPTSearch(VnsSolver* solver,
                                  int opt,
                                  uint64 time_limit,
                                  int log,
                                  ProblemSolution* sol) {
	// to keep the time
  Stopwatch^ sw = gcnew Stopwatch();

  int TL = time_limit / 1000.0;
  double UB = sol->cost() >= 0 ? sol->cost() : Globals::Infinity();
  SolverOptions options;
  options.set_cut_off_value(UB);
  options.set_max_time(TL);
  SolverStatus status;

  solver->AddConsMaxAssignmentChanges(*sol, opt);
  solver->Solve(options, &status);
  if (status.status == OPTSTAT_FEASIBLE || status.status == OPTSTAT_MIPOPTIMAL) {
    VLOG(log) << "SimpleOPTSearch: found feasible solution, cost: "
              << status.final_sol.cost();
    *sol = status.final_sol;
  } else
    VLOG(log) << "SimpleOPTSearch: fail with status " << status.status;
}

void LocalSearch::GetEllipsoidalBounds(const vector<const ProblemSolution*>& sols,
                                       int* minimum_lhs, int* maximum_lhs) {
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

  *maximum_lhs = N * sols.size() - min_feasible_dist;
  *minimum_lhs = N * sols.size() - max_feasible_dist;
}

bool LocalSearch::EllipsoidalSearch(VnsSolver* solver,
                                    const vector<const ProblemSolution*>& sols,
                                    uint64 total_time_ms,
                                    uint64 step_time_ms,
                                    int log,
                                    int num_solutions,
                                    uint64* time_elapsed_ms,
                                    vector<ProblemSolution*>* final_sols,
                                    double initial_UB) {

#define RHS(F) (Globals::instance()->n() * sols.size() - (F))
  // to keep the time
	*time_elapsed_ms = 0;
  Stopwatch^ sw = gcnew Stopwatch();

  // First, calculates the minimum and maximum slack possible
  int min_opt, max_opt;
  GetEllipsoidalBounds(sols, &min_opt, &max_opt);

  double UB = initial_UB;
  if (initial_UB >= 1e18 - 1.0) {
    if (num_solutions > 0) {
      UB = 0.0;
      for (int i = 0; i < sols.size(); ++i)
        UB += sols[i]->cost();
      UB /= sols.size();
    } else {
      for (int i = 0; i < sols.size(); ++i)
        UB = min<double>(UB, sols[i]->cost());
    }
  }

  bool feasible = false;
  int rhs = min_opt + 2;
  int k_step = 16;
  deque<int> added_cons;

  bool cont = true;
  while (cont && *time_elapsed_ms < total_time_ms && rhs + k_step <= max_opt) {
    cont = false;
    uint64 TL = (*time_elapsed_ms + step_time_ms > total_time_ms ?
                 total_time_ms - *time_elapsed_ms : step_time_ms);
    VLOG(log) << "EllipsoidalSearch: adding constraint " << rhs << " <= SUM <= "
              << rhs + k_step << ", UB: " << UB << ", time limit: " << TL / 1000.0;

    // Adds constraint
    added_cons.push_back(
        solver->AddEllipsoidalConstraintMinDistance(sols, RHS(rhs)));
    added_cons.push_back(
        solver->AddEllipsoidalConstraintMaxDistance(sols, RHS(rhs + k_step)));

    // Runs the solver
    sw->Start();
    PopulateOptions options;
    options.set_max_time(TL / 1000.0);
    options.set_cut_off_value(UB);
    options.set_num_max_solutions(num_solutions);
    //options.set_num
    PopulateStatus status;
    if (num_solutions >= 1)
		  solver->Populate(options, &status);
    else
      solver->Solve(options, &status);
		sw->Stop();
		*time_elapsed_ms += sw->ElapsedMilliseconds;
		sw->Reset();

    switch (status.status) {
      case OPTSTAT_MIPOPTIMAL:
      case OPTSTAT_FEASIBLE:
        CHECK_LT(status.solution_set[0].cost(), UB);
        for (int i = 0; i < status.solution_set.size(); ++i) {
          ProblemSolution* new_sol = new ProblemSolution(status.solution_set[i]);
          final_sols->push_back(new_sol);
        }
        if (status.solution_set.size() == 0)
          final_sols->push_back(new ProblemSolution(status.final_sol));
        VLOG(log) << "EllipsoidalSearch: optimal/feasible for rhs: " << rhs << ", cost : "
                  << status.final_sol.cost() << ", time elapsed: " << *time_elapsed_ms
                  << ", best solution: " << status.final_sol.cost();
        feasible = cont = true;
        UB = status.final_sol.cost();
        break;
      case OPTSTAT_INFEASIBLE:
        VLOG(log) << "EllipsoidalSearch: infeasible for rhs: " << rhs << ", time elapsed: "
                  << *time_elapsed_ms << ", new rhs: " << rhs + 1;
        break;
      case OPTSTAT_NOINTEGER:
        VLOG(log) << "EllipsoidalSearch: no integer solution for rhs:  " << rhs;
        break;
    }

    rhs += k_step;

    while (!added_cons.empty()) {
      VLOG(log + 1) << "EllipsoidalSearch: removing constraint: " << added_cons.back();
      solver->RemoveConstraint(added_cons.back());
      added_cons.pop_back();
    }
	}

  if (feasible)
    VLOG(log) << "EllipsoidalSearch: success, time spent: " << *time_elapsed_ms / 1000.0;
  else
    VLOG(log) << "EllipsoidalSearch: fail, time spent: " << *time_elapsed_ms / 1000.0;

	return feasible;
#undef RHS
}

void LocalSearch::MultiEllipsoidalSearch() {
	vector<ProblemSolution> pool(5, ProblemSolution(Globals::instance()));
	vector<ProblemSolution> opt_pool(30, ProblemSolution(Globals::instance()));
	for (int i = 0; i < static_cast<int>(pool.size()); i++) {
		ConstructiveHeuristics::RandomStupid(*Globals::instance(), &pool[i]);
		//LocalSearch::SimpleOPTSearch(pool[i], 2, 1, 30, 5, 5, &opt_pool[i]);
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
  CHECK(status->status == OPTSTAT_FEASIBLE || status->status == OPTSTAT_MIPOPTIMAL);
  if (status->status == OPTSTAT_MIPOPTIMAL)
    return;
  x_opt = status->final_sol;  // creates initial solution

  ProblemSolution x_cur(x_opt);
  double f_cur = x_opt.cost();
  while (elapsed_ms < total_time_ms) {
    LOG(INFO) << "Iteration " << ++iter << " - elapsed time: "
              << static_cast<double>(elapsed_ms) / 1000.0 << "s" << endl;
    uint64 time_to_best = 0;
		uint64 intensif_time =
        VNSIntensification(solver_intensification, 2, 1,
                           total_time_ms - static_cast<int>(elapsed_ms),
                           node_time_ms, 0, &time_to_best, &x_cur);

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
        solver_diversification->AddConsMinAssignmentChanges(x_opt, k_cur + k_step);

      // add constraint delta(x, x_opt) <= k_cur + k_step
      int last_cons_greater =
        solver_diversification->AddConsMaxAssignmentChanges(x_opt, k_cur);

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
                                       int initial_opt,
                                       int step_opt,
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
  int rhs = initial_opt;
  while (cont && elapsed_time < total_time_ms && rhs < Globals::instance()->n()) {
    int TL = min(node_time_ms, total_time_ms - static_cast<int>(elapsed_time));
    VLOG_EVERY_N(log + 1, 20)
        << "VNSInt: Adding local branching constraint delta(x, x_cur) "
        << "<= rhs = " << rhs << " (>= " << RHS(rhs) << "), UB = f_cur = "
        << x_cur->cost() << ", time: " << TL / 1000.0 << ", num constr: "
        << added_cons.size();
    added_cons.push_back(
        solver_intensification->AddConsMaxAssignmentChanges(*x_cur, RHS(rhs)));
    double UB = x_cur->cost();

    // runs CPLEX
    sw->Start();
    // status = MIPSOLVE(TL, UB, first = false, x_next, f_next)
    SolverOptions options; options.set_max_time(TL / 1000); options.set_cut_off_value(UB);
    SolverStatus status;
    solver_intensification->Solve(options, &status);
    VLOG_EVERY_N(log, 20)
        << "VNSInt: status = MIPSOLVE(TL=" << options.max_time() << ", UB="
        << UB << ", first=false, x_next, f_next) = " << status.status;
    sw->Stop();
    elapsed_time += sw->ElapsedMilliseconds;
    sw->Reset();
    
    switch(status.status) {
      case OPTSTAT_MIPOPTIMAL:
        VLOG(log + 1) << "VNSInt: optimal, reverting constraint to delta(x, x_cur) >= "
                  << rhs + 1 << " (<= " << RHS(rhs + 1) << ")";
        // reverse last local branching constraint into delta(x, x_cur) >= rhs + 1
        solver_intensification->ReverseConstraint(added_cons.back(), RHS(rhs + 1));
        // x_cur = x_next, f_cur = f_next;
        CHECK_LT(status.final_sol.cost(), x_cur->cost());
        *time_to_sol = elapsed_time;
        *x_cur = status.final_sol;
        VLOG(log) << "VNSInt: new best solution (optimal), cost: " << x_cur->cost();
        rhs = initial_opt;
        break;
      case OPTSTAT_FEASIBLE:
        VLOG(log + 2)  << "VNSInt: feasible, reverting constraint to delta(x, x_cur) >= "
                       << 2 << " (<= " << RHS(2) << ")";
        // reverse last local branching constraint into delta(x, x_cur) >= 1
        solver_intensification->ReverseConstraint(added_cons.back(), RHS(2));
        //x_cur = x_next, f_cur = f_next;
        CHECK_LT(status.final_sol.cost(), x_cur->cost());
        *time_to_sol = elapsed_time;
        *x_cur = status.final_sol;
        VLOG(log) << "VNSInt: new best solution (feasible), cost: " << x_cur->cost();
        rhs = initial_opt;
        break;
      case OPTSTAT_INFEASIBLE:
        VLOG(log + 2)  << "VNSInt: proven infeasible, removing last constraint, "
                  << "new RHS = " << rhs + 1 << " (>= " << RHS(rhs + 1) << ")";
        // remove last local branching constraint;
        solver_intensification->RemoveConstraint(added_cons.back());
        added_cons.pop_back();
        rhs += step_opt;
        break;
      case OPTSTAT_NOINTEGER:
        VLOG(log)  << "VNSInt: no integer solution, breaking out of the search";
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

  VLOG(log) << "VNSInt: finished, returning time " << elapsed_time << ", cost:  " << x_cur->cost();
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
				2,
        1,
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

void LocalSearch::PickNSolutions(int N, int log, const FixedSizeSolutionSet& R,
                                 vector<const ProblemSolution*>* sols) {
  sols->reserve(N);
  set<int> sol_indices;
  while (sol_indices.size() < N) {
    int sol_to_insert = Globals::rg()->IRandom(0, R.size() - 1);
    if (sol_indices.insert(sol_to_insert).second) {
      sols->push_back(R.GetSolutionPtr(sol_to_insert));
      VLOG(log)
          << "Selecting solution " << sol_to_insert << ", cost: "
          << sols->at(sols->size() - 1)->cost();
    }
  }
}

void LocalSearch::PathRelink(SolverFactory* solver_factory,
                             uint64 total_time_ms,
                             uint64 local_search_time_ms,
                             int log,
                             SolverStatus* final_status) {
  Stopwatch^ sw = gcnew Stopwatch();
  uint64 elapsed_time = 0;
  uint64 time_to_best = 0;

  // Gerar um conjunto de soluções aleatórias em R
  const int kSetSize = 10;
  FixedSizeSolutionSet R(kSetSize);
  {
    VnsSolver* solver = solver_factory->NewVnsSolver(Globals::instance());
    PopulateOptions options;
    options.set_max_time(local_search_time_ms * 5 / 1000.0);
    options.set_num_max_solutions(200);
    //options.set_num_max_solutions(kSetSize);
    PopulateStatus status;
    solver->Init(options);
    sw->Start();
    solver->Populate(options, &status);
    sw->Stop(); elapsed_time += sw->ElapsedMilliseconds; sw->Reset();

    CHECK(status.status == OPTSTAT_FEASIBLE || status.status == OPTSTAT_MIPOPTIMAL);
    final_status->final_sol = status.final_sol;
    time_to_best = elapsed_time;

    for (int sol = 0; sol < status.solution_set.size(); ++sol) {
      VLOG(log) << "PathRelink: adding initial solution " << sol << " (from CPLEX), cost: "
                << status.solution_set[sol].cost();
      R.AddSolution(status.solution_set[sol]);
      if (status.solution_set[sol].cost() < final_status->final_sol.cost()) {
        final_status->final_sol = status.solution_set[sol];
        time_to_best = elapsed_time;
      }
    }

    for (int sol = status.solution_set.size(); sol < kSetSize; ++sol) {
      sw->Start();
      ProblemSolution first_solution = status.final_sol;
      RandomlyMutateSolution(Globals::rg()->IRandom(30, 60), &first_solution);
      uint64 local_time_to_best = 0;
      LocalSearch::SimpleOPTSearch(solver, 10, local_search_time_ms,
                                   log + 1, &first_solution);
      //LocalSearch::VNSIntensification(
      //    solver, 10, 10, local_search_time_ms,
      //    local_search_time_ms / 2, log + 1, &local_time_to_best, &first_solution);
      R.AddSolution(first_solution);
      if (first_solution.cost() < final_status->final_sol.cost()) {
        final_status->final_sol = first_solution;
        time_to_best = elapsed_time + local_time_to_best;
      }

      VLOG(log) << "PathRelink: adding initial solution " << sol
                << " (from local search), cost: " << first_solution.cost();
      sw->Stop(); elapsed_time += sw->ElapsedMilliseconds; sw->Reset();
    }

    delete solver;
  }

  // Enquanto criterio de parada nao for atingido
  VnsSolver* solver_diver = solver_factory->NewVnsSolver(Globals::instance());
  solver_diver->Init(SolverOptions());
  while (elapsed_time < total_time_ms) {
    // Um conjunto de soluções a serem otimizadas
    vector<ProblemSolution*> S;

    // Achar as X melhores soluções no caminho de A para B, adicionar a S
    const int X = 18;
    while (S.size() < X && elapsed_time < total_time_ms) {
      VLOG(log) << "PathRelink: relink step, " << X - S.size() << " still remaining.";
      // Escolher aleatóriamente duas soluções de R
      const int kNumSolEllipsoidal = Globals::rg()->IRandom(3, 5);
      vector<const ProblemSolution*> sol_in_ellipse;
      PickNSolutions(kNumSolEllipsoidal, log + 1, R, &sol_in_ellipse);

      // TODO(danielrocha): talvez fazer mutacao nas solucoes?

      uint64 time_elapsed_ms = 0;
      vector<ProblemSolution*> new_solutions;
      sw->Start();
      bool feasible =
          LocalSearch::EllipsoidalSearch(
              solver_diver, sol_in_ellipse, total_time_ms - elapsed_time,
              local_search_time_ms, log, 4, &time_elapsed_ms, &S);
      sw->Stop(); elapsed_time += sw->ElapsedMilliseconds; sw->Reset();
      for (int i = S.size() - 4; S.size() >= 4 && i < S.size(); ++i) {
        if (S[i]->cost() < final_status->final_sol.cost()) {
          final_status->final_sol = *S[i];
          time_to_best = elapsed_time;
        }
      }

      if (feasible) {
        VLOG(log) << "PathRelink: inserted result of path relink into S, num solutions: "
                  << S.size();
      } else {
        VLOG(log) << "PathRelink: path relink infeasible, S size: " << S.size();
      }
    }

    // Enquanto existirem soluções a ser otimizadas em S:
    VnsSolver* solver_inten = solver_factory->NewVnsSolver(Globals::instance());
    solver_inten->Init(SolverOptions());
    while (S.size() > 0 && elapsed_time < total_time_ms) {
      // Escolher uma solução aleatória, otimizar, remover de S
      int picked_sol = Globals::rg()->IRandom(0, S.size() - 1);
      ProblemSolution* s = S[picked_sol];
      uint64 local_time_to_best = 0;
      VLOG(log) << "PathRelink: intensifying solution from S, current cost: " << s->cost()
                << ", position: " << picked_sol << ", S size: " << S.size();
      sw->Start();
      LocalSearch::SimpleOPTSearch(solver_inten, 10, local_search_time_ms / 2, log + 1, s);
      //LocalSearch::VNSIntensification(
      //    solver_inten, 10, 10, local_search_time_ms,
      //    local_search_time_ms / 2, log + 1, &local_time_to_best, s);

      // Tentar adicionar o ótimo local a R
      if (R.AddSolution(*s)) {
        if (s->cost() < final_status->final_sol.cost()) {
          final_status->final_sol = *s;
          time_to_best = elapsed_time + local_time_to_best;
        }
        VLOG(log) << "PathRelink: added local optima to R, new cost: " << s->cost();
      } else {
        VLOG(log) << "PathRelink: DIDN'T add local optima to R, new cost: " << s->cost();
      }
      sw->Stop(); elapsed_time += sw->ElapsedMilliseconds; sw->Reset();

      // Remove a solucao de S - transfere pro final e da um pop_back
      swap(S[picked_sol], S[S.size() - 1]);
      delete S[S.size() - 1];
      S.pop_back();

    }
    delete solver_inten;
    VLOG(log) << "PathRelink: best solution so far: " << final_status->final_sol.cost()
              << ", elapsed time: " << elapsed_time / 1000.0;
  }
  delete solver_diver;
  final_status->str_status = "heuristic";
  final_status->time_to_best_solution = time_to_best / 1000.0;

  VLOG(log) << "PathRelink: starting post processing";
  VnsSolver* solver = solver_factory->NewVnsSolver(Globals::instance());
  solver->Init(SolverOptions());
  ProblemSolution best_result(final_status->final_sol);
  for (int cont = 0; cont <= 3; ++cont) {
    vector<const ProblemSolution*> sols;
    vector<ProblemSolution*> result;
    PickNSolutions(Globals::rg()->IRandom(4, 6), log + 1, R, &sols);
    sw->Start();
    uint64 time = 0;
    bool feasible = LocalSearch::EllipsoidalSearch(
        solver, sols, total_time_ms,
        local_search_time_ms, log, -1, &time, &result);
    sw->Stop(); elapsed_time += sw->ElapsedMilliseconds; sw->Reset();
    if (feasible && result.size() >= 1) {
      VLOG(log) << "PathRelink: post-processing step " << cont << ", best solution: "
                << sols[0]->cost();
      if (result[0]->cost() < best_result.cost()) {
        VLOG(log) << "PathRelink: post-processing step found BETTER solution: "
                  << result[0]->cost() << ", time: "
                  << (elapsed_time + time) / 1000.0;
        best_result = (*result[0]);
        //time_to_best = elapsed_time + time;
      }
    } else {
      VLOG(log) << "PathRelink: post-processing step " << cont << ", failed.";
    }
  }
  delete solver;
  VLOG(log) << "PathRelink: done, final result: " << final_status->final_sol.cost()
            << ", time to best: " << time_to_best / 1000.0 << ", total time: " << elapsed_time; 
}

void LocalSearch::CalculateDistances(int num_sols, const FixedSizeSolutionSet& R,
                                     const vector<vector<int> >& distance,
                                     vector<pair<int, vector<int> > >* sol_indices) {
  vector<int> v;
  for (int sol1 = 0; sol1 < R.size(); ++sol1) {
    v.push_back(sol1);
    for (int sol2 = sol1 + 1; sol2 < R.size(); ++sol2) {
      int dist2 = distance[sol1][sol2];
      v.push_back(sol2);
      if (num_sols == 2) {
        sol_indices->push_back(make_pair(dist2, v));
        continue;
      }
      for (int sol3 = sol2 + 1; sol3 < R.size(); ++sol3) {
        int dist3 = dist2 + distance[sol1][sol3] + distance[sol2][sol3];
        v.push_back(sol3);
        if (num_sols == 3) {
          sol_indices->push_back(make_pair(dist3, v));
          continue;
        }
        for (int sol4 = sol3 + 1; sol4 < R.size(); ++sol4) {
          int dist4 = (dist3 + distance[sol1][sol4] + distance[sol2][sol4] +
                       distance[sol3][sol4]);
          v.push_back(sol4);
          sol_indices->push_back(make_pair(dist4, v));
          v.pop_back();
        }
        v.pop_back();
      }
      v.pop_back();
    }
    v.pop_back();
  }
  sort(sol_indices->rbegin(), sol_indices->rend());
}

void LocalSearch::PostProcessing(SolverFactory* solver_factory,
                                 uint64 total_time_ms,
                                 uint64 local_search_time_ms,
                                 int log,
                                 SolverStatus* final_status) {
  uint64 elapsed_time = 0;
  Stopwatch^ sw = gcnew Stopwatch();
  FixedSizeSolutionSet R(20);

  // Calculates the remaining time for the post processing
  const int K = 15;
  uint64 cplex_time = total_time_ms - K * local_search_time_ms;
  VnsSolver* solver = solver_factory->NewVnsSolver(Globals::instance());
  solver->Init(SolverOptions());
  {
    // Runs CPLEX + Populate
    PopulateOptions options;
    options.set_max_time(cplex_time / 1000.0);
    options.set_num_max_solutions(100);
    PopulateStatus status;
    VLOG(log) << "PostProcessing: calling solver with TL = "
              << cplex_time / 1000.0;
    solver->Init(options);
    sw->Start();
    solver->Populate(options, &status);
    sw->Stop(); elapsed_time += sw->ElapsedMilliseconds; sw->Reset();
    if (status.status == OPTSTAT_FEASIBLE || status.status == OPTSTAT_MIPOPTIMAL) {
      for (int i = 0; i < status.solution_set.size(); ++i) {
        if (i == 0 || status.solution_set[i].cost() < final_status->final_sol.cost()) {
          final_status->final_sol = status.solution_set[i];
          final_status->time_to_best_solution = elapsed_time / 1000.0;
        }
        if (R.AddSolution(status.solution_set[i]))
          VLOG(log) << "PostProcessing: added solution " << R.size() << ", cost: "
                    << status.solution_set[i].cost();
      }
      VLOG(log) << "PostProcessing: finished solver with status = "
          << status.status << ", best solution "
          << final_status->final_sol.cost();
    } else {
      VLOG(log) << "PostProcessing: FAIL!!";
      final_status->status = OPTSTAT_INFEASIBLE;
      return;
    }
  }
  VLOG(log) << "PostProcessing: starting recombination step.";

  vector<vector<int> > distance(R.size(), vector<int>(R.size(), 0));
  for (int i = 0; i < R.size(); ++i)
    for (int j = i + 1; j < R.size(); ++j)
      distance[i][j] = distance[j][i] = R.GetSolution(i).Distance(R.GetSolution(j));

  while (elapsed_time < total_time_ms) {
    for (int num_sols = 2; num_sols <= 4; ++num_sols) {

      vector<pair<int, vector<int> > > sol_indices;
      LocalSearch::CalculateDistances(num_sols, R, distance, &sol_indices);

      for (int repet = 0; repet < K / (4 - 2 + 1); ++repet) {
        vector<const ProblemSolution*> sols;
        vector<ProblemSolution*> result;
        //LocalSearch::PickNSolutions(num_sols, log, R, &sols);
        for (int i = 0; i < num_sols; ++i) {
          VLOG(log) << "PostProcessing: picking solution: " << sol_indices[repet].second[i]
                    << ", cost: " << R.GetSolution(sol_indices[repet].second[i]).cost()
                    << ", total distance: " << sol_indices[repet].first;
          sols.push_back(R.GetSolutionPtr(sol_indices[repet].second[i]));
        }

        sw->Start();
        uint64 time = 0;
        bool feasible = LocalSearch::EllipsoidalSearch(
            solver, sols, total_time_ms,
            local_search_time_ms, log, -1, &time, &result, final_status->final_sol.cost());
        sw->Stop(); elapsed_time += sw->ElapsedMilliseconds; sw->Reset();
        if (feasible && result.size() >= 1) {
          VLOG(log) << "PostProcessing: num_sols: " << num_sols << ", repet: " << repet
                    << " best solution: " << result[0]->cost();
          if (result[0]->cost() < final_status->final_sol.cost()) {
            VLOG(log) << "PostProcessing: found BEST solution, cost: "
                      << result[0]->cost();
            final_status->final_sol = *(result[0]);
            final_status->time_to_best_solution = elapsed_time / 1000.0;
          }
          delete result[0];
        } else {
          VLOG(log) << "PostProcessing: num_sols: " << num_sols << ", repet: " << repet
                    << " fail.";
        }
      }

      VLOG(log) << "PostProcessing: finished num_sols: " << num_sols << " best solution: "
                << final_status->final_sol.cost() << ", time elapsed: "
                << elapsed_time / 1000.0;
    }
  }
  delete solver;
}