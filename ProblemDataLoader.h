#pragma once

#include <string>
#include "ProblemData.h"

using namespace std;

class ProblemData;

class ProblemDataLoader
{
public:
    ProblemDataLoader() {}
    ProblemDataLoader(const char* inputfile, ProblemData* pdt);
    ~ProblemDataLoader(void);
    void load();
private:
    string file;
    ProblemData* pd;
};
