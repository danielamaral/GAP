// GAP.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <iostream>

#include "ProblemData.h"
#include "ProblemDataLoader.h"
#include "ProblemSolution.h"
#include "Solver.h"
#include "SolverFormulacaoPadrao.h"

#define LOG(INFO) std::cerr

const int kNumberArguments = 2;

ProblemData instance;
ProblemSolution ps;

int main(int argc, char* argv[])
{
    if (argc < kNumberArguments) {
        LOG(INFO) << "Error loading instance file, not enough command line arguments" << endl;
        LOG(INFO) << "Usage: " << argv[0] << " INSTANCE_FILE" << endl;
        return 1;
    }

    ProblemDataLoader loader(argv[1], &instance);
    loader.load();

    SolverFormulacaoPadrao solver(&instance);
    solver.solve();
    solver.generate_solution(&ps);
    cout << ps;
	return 0;
}

