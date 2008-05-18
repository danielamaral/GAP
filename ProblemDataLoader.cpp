#include "ProblemDataLoader.h"

#include <cassert>
#include <fstream>
#include "ProblemData.h"

ProblemDataLoader::ProblemDataLoader(const char* filename, int optimal, ProblemData* pdt) : file(filename), pd(pdt), opt(optimal)
{
}

ProblemDataLoader::~ProblemDataLoader(void)
{
}

void ProblemDataLoader::load() {
    ifstream f(file.c_str(), ifstream::in);
    assert(f.good());

    int M, N;
    f >> M >> N;
    pd->set_m(M);
    pd->set_n(N);
    pd->set_optimal(opt);

    int cost;
    for (int machine = 0; machine < pd->m(); ++machine) {
        for (int task = 0; task < pd->n(); ++task) {
            f >> cost;
            pd->set_cost(machine, task, cost);
        }
    }

    int consume;
    for (int machine = 0; machine < pd->m(); ++machine) {
        for (int task = 0; task < pd->n(); ++task) {
            f >> consume;
            pd->set_consume(machine, task, consume);
        }
    }

    int capacity;
    for (int machine = 0; machine < pd->m(); ++machine) {
        f >> capacity;
        pd->set_capacity(machine, capacity);
    }

    f.close();
}