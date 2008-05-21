// GAP.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#using <System.dll>
using namespace System::Diagnostics;

#include <iostream>

#include "ConstructiveHeuristics.h"
#include "Globals.h"
#include "LocalSearch.h"
#include "ProblemData.h"
#include "ProblemDataLoader.h"
#include "ProblemSolution.h"
#include "Solver.h"
#include "SolverFormulacaoPadrao.h"

int main(int argc, char* argv[])
{
    const int kNumberArguments = 5;
    if (argc < kNumberArguments) {
        cout << "Error loading instance file, not enough command line arguments" << endl;
        cout << "Usage: " << argv[0] << " INSTANCE_FILE RANDOM_SEED UPPER_BOUND ALGORITHM" << endl;
        return 1;
    }
	string instance_file(argv[1]);
	int random_seed = atoi(argv[2]);
	int upper_bound = atoi(argv[3]);
	string algorithm(argv[4]);

    Globals::rg()->RandomInit(random_seed);

    ProblemDataLoader loader(instance_file.c_str(), upper_bound, Globals::instance());
    loader.load();
    cout << "Loading input..." << endl;

	//ConstructiveHeuristics::RandomStupid(*Globals::instance(), &init_sol1);
	//ConstructiveHeuristics::RandomStupid(*Globals::instance(), &init_sol2);
    //LocalSearch::SimpleOPTSearch(init_sol1, 20, 60, 16, &final_sol);
	//LocalSearch::EllipsoidalSearch(init_sol1, init_sol2, 5, &final_sol);
	//LocalSearch::MultiEllipsoidalSearch();
    //cout << "Best solution: " << final_sol.cost() << endl;

    // to keep the time
    Stopwatch^ sw = gcnew Stopwatch();
	// final solution
	ProblemSolution final_sol(Globals::instance());
	if (algorithm == "CPLEX") {
		sw->Start();
		SolverFormulacaoPadrao solver(Globals::instance());
		if (solver.Solve(60 * 60) == 0) {
			sw->Stop();
			ProblemSolution sol(Globals::instance());
			solver.GenerateSolution(&final_sol);
			cout << final_sol.cost() << " " << static_cast<double>(sw->ElapsedMilliseconds) / 1000.0 << endl;
		}
	} else if (algorithm == "CPLEX-UB") {
		sw->Start();
		SolverFormulacaoPadrao solver(Globals::instance());
		if (solver.SolveTLAndUB(60 * 60, upper_bound) == 0) {
			sw->Stop();
			ProblemSolution sol(Globals::instance());
			solver.GenerateSolution(&final_sol);
			cout << final_sol.cost() << " " << static_cast<double>(sw->ElapsedMilliseconds) / 1000.0 << endl;
		}
	} else if (algorithm == "VNSBra") {
		sw->Start();
		LocalSearch::VNSBra(60 * 60 * 1000, 120 * 1000, 5, &final_sol);
		sw->Stop();
		cout << final_sol.cost() << " " << static_cast<double>(sw->ElapsedMilliseconds) / 1000.0 << endl;
	} else if (algorithm == "Memetic") {
		sw->Start();
		LocalSearch::MIPMemetic(&final_sol);
		sw->Stop();
		cout << final_sol.cost() << " " << static_cast<double>(sw->ElapsedMilliseconds) / 1000.0 << endl;
	}
	return 0;
}

