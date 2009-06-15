// GAP.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#using <System.dll>
using namespace System::Diagnostics;

#include <iostream>

#include "ConstructiveHeuristics.h"
#include "Globals.h"
#include "LocalSearch.h"
#include "logging.h"
#include "opt_cplex.h"
#include "ProblemData.h"
#include "ProblemDataLoader.h"
#include "ProblemSolution.h"
#include "Solver.h"
#include "SolverFormulacaoPadrao.h"
#include "SolverGeracaoColunas.h"

void ReadAndSolveMpsFile(const string& input_file) {
  OPT_CPLEX* lp = new OPT_CPLEX;
  lp->readCopyMPS(const_cast<char*>(input_file.c_str()));
  lp->setSimplexScreenLog(1);
  const int kNumLpMethods = 7;
  METHOD kMethodsToTry[kNumLpMethods] = {
    METHOD_AUTOMATIC, METHOD_SIFTING, METHOD_PRIMAL, METHOD_CONCURRENT, METHOD_DUAL,
    METHOD_BARRIERANDCROSSOVER, METHOD_BARRIERNOCROSSOVER };

  OPTSTAT status;
  for (int method = 0; method < kNumLpMethods; ++method) {
    LOG(INFO) << "Trying method " << method << "...";
    status = lp->optimize(kMethodsToTry[method]);
    LOG(INFO) << "Method " << method << ": status " << status
            << " obj " << lp->getObjVal();
    if (status == OPTSTAT_LPOPTIMAL || status == OPTSTAT_MIPOPTIMAL)
      break;
  }

  delete lp;
}

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);

  const int kNumberArguments = 6;
  if (argc < kNumberArguments) {
    LOG(ERROR) << "Usage: " << argv[0] << " "
               << "INSTANCE_FILE SOLVER RANDOM_SEED "
               << "ALGORITHM MAX_TIME [UPPER_BOUND] [SOLVER_LOG_LEVEL]";
    return argc;
  }

	string instance_file(argv[1]);
  string formulacao(argv[2]);
	int random_seed = atoi(argv[3]);
	string algorithm(argv[4]);
  int max_time = atoi(argv[5]);
  int upper_bound = (argc >= 7 ? atoi(argv[6]) : 0);
  if (argc >= 8)
    Globals::SetSolverLog(atoi(argv[7]));

  if (instance_file.find(".mps") != string::npos) {
    ReadAndSolveMpsFile(instance_file);
    return 0;
  }

  Globals::rg()->RandomInit(random_seed);
  VLOG(1) << "Loading instance file: " << instance_file;
  ProblemDataLoader loader(instance_file.c_str(), upper_bound, Globals::instance());
  loader.load();

  // Solver options
  SolverOptions options;
  options.set_max_time(max_time);
  options.set_relative_time_for_first_solution(Globals::instance()->n() *
                                               Globals::instance()->m());
  if (upper_bound > 0)
    options.set_cut_off_value(upper_bound);
  options.set_use_stabilization(true);

	// Solver and input options
  Solver* solver;
  SolverFactory* solver_factory;
  if (formulacao == "GeracaoColunas") {
    solver = new SolverGeracaoColunas(Globals::instance());
    solver_factory = new SolverGeracaoColunasFactory();
  } else if (formulacao == "FormulacaoPadrao") {
    solver = new SolverFormulacaoPadrao(Globals::instance());
    solver_factory = new SolverFormulacaoPadraoFactory();
  } else {
    LOG(FATAL) << "Inexistent formulation specified: " << formulacao;
    return 2;
  }

  // to keep the time
  Stopwatch^ sw = gcnew Stopwatch();

	// input options and final status
	SolverStatus status;
  //PopulateStatus status;

	if (algorithm == "CPLEX") {
		sw->Start();
		solver->Init(options);
    //solver->Populate(options, &status);
		solver->Solve(options, &status);
		sw->Stop();
	} else if (algorithm == "CPLEX-UB") {
    options.set_cut_off_value(upper_bound);
		sw->Start();
		solver->Init(options);
		solver->Solve(options, &status);
		sw->Stop();
	} else if (algorithm == "VNSBra") {
    options.set_time_for_first_solution(60);
		sw->Start();
    LocalSearch::VNSBra(solver_factory, options.max_time() * 1000,
                        120 * 1000, 5, &status);
		sw->Stop();
	} else if (algorithm == "Memetic") {
		sw->Start();
		LocalSearch::MIPMemetic(&status);
		sw->Stop();
  } else if (algorithm == "PathRelink") {
    sw->Start();
    LocalSearch::PathRelink(solver_factory, options.max_time() * 1000,
                            120 * 1000, 1, &status);
    sw->Stop();
  } else {
    LOG(FATAL) << "Inexistent algorithm specified: " << algorithm;
    return 3;
  }

  status.time = sw->ElapsedMilliseconds / 1000.0;
	LOG(INFO) << status.ToString();
	return 0;
}