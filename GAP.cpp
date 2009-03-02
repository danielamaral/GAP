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
#include "SolverGeracaoColunas.h"

int main(int argc, char* argv[])
{
  const int kNumberArguments = 6;
  if (argc < kNumberArguments) {
      cout << "Error loading instance file, not enough command line arguments" << endl;
      cout << "Usage: " << argv[0] << " INSTANCE_FILE RANDOM_SEED UPPER_BOUND ALGORITHM VERBOSE_LEVEL" << endl;
      return 1;
  }
	string instance_file(argv[1]);
	int random_seed = atoi(argv[2]);
	int upper_bound = atoi(argv[3]);
	string algorithm(argv[4]);
	int verbosity = atoi(argv[5]);

  Globals::rg()->RandomInit(random_seed);

  ProblemDataLoader loader(instance_file.c_str(), upper_bound, Globals::instance());
  loader.load();
  cout << "Loading input..." << endl;

	// Solver
	//SolverFormulacaoPadrao solver(Globals::instance());
  SolverGeracaoColunas solver(Globals::instance());

  // to keep the time
  Stopwatch^ sw = gcnew Stopwatch();

	// final solution
	ProblemSolution final_sol(Globals::instance());

	// final status
	SolverStatus status;

	if (algorithm == "CPLEX") {
		sw->Start();
		solver.Solve(60 * 60, &status);
		sw->Stop();
	} else if (algorithm == "CPLEX-UB") {
		sw->Start();
		SolverFormulacaoPadrao solver(Globals::instance());
		solver.Init();
		solver.Solve(60 * 60, &status);
		sw->Stop();
	} else if (algorithm == "VNSBra") {
		sw->Start();
		LocalSearch::VNSBra(60 * 60 * 1000, 120 * 1000, 5, &status);
		sw->Stop();
		//cout << final_sol.cost() << " " << static_cast<double>(sw->ElapsedMilliseconds) / 1000.0 << endl;
	} else if (algorithm == "Memetic") {
		sw->Start();
		LocalSearch::MIPMemetic(&status);
		sw->Stop();
		//cout << final_sol.cost() << " " << static_cast<double>(sw->ElapsedMilliseconds) / 1000.0 << endl;
	}

	cout << status.ToString()<< endl;
	return 0;
}

