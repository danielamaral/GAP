#ifndef GLOBALS_H
#define GLOBALS_H

#include <cstdio>

#include "ProblemData.h"
#include "randomc.h"

class Globals {
public:
  static ProblemData* instance() {
    if (instance_ == NULL) {
      instance_ = new ProblemData();
    }
    return instance_;
  }
  static CRandomMersenne* rg() {
    if (rg_ == NULL) {
      rg_ = new CRandomMersenne(0);
    }
    return rg_;
  }
  static double Infinity() { return kInfinity_; };
  static double EPS() { return Epsilon_; };
  static double BigEPS() { return BigEpsilon_; }
  static int SolverLog() { return SolverLog_; }
  static void SetSolverLog(int x) { SolverLog_ = x; }
private:
  Globals();
  Globals(const Globals&);
  Globals& operator=(const Globals&);

  static CRandomMersenne* rg_;
  static ProblemData* instance_;
	static double kInfinity_;
  static double Epsilon_;
  static double BigEpsilon_;
  static int SolverLog_;
};

enum FixingSense {
  FIX_ON_ZERO = 0,
  FIX_ON_ONE
};

#endif