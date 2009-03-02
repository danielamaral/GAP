#include "ConstructiveHeuristics.h"

#include <iostream>
#include "Globals.h"
#include "ProblemData.h"
#include "ProblemSolution.h"

namespace ConstructiveHeuristics {
    void RandomStupid(const ProblemData& pd, ProblemSolution* sol) {
        // Random stupid way
        // for each task
restart:
		std::cout << "attempting" << std::endl;
		sol->Clear();
        for (int i = 0; i < pd.n(); ++i) {
            // randomly select a machine
            int machine;
			int cont = 0;
            do {
                machine = Globals::rg()->IRandom(0, pd.m() - 1);
				//std::cout << "task = " << i << " machine = " << machine << " capacity - used = " << pd.capacity(machine) - sol->used(machine) << std::endl;
				++cont;
				if (cont > 1000) goto restart;
            } while (pd.capacity(machine) - sol->used(machine) < pd.consume(machine, i));
            std::cout << i << std::endl;
            sol->set_assignment(i, machine);
        }
    }
};