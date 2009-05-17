#ifndef VNSCUT_H
#define VNSCUT_H

#include <vector>

#include "Globals.h"
#include "ProblemSolution.h"

class VnsCut {
public:
  VnsCut()
      : max_changes_(-1), min_changes_(-1),
        solution_(ProblemSolution(Globals::instance())) {}
  int max_changes() const { return max_changes_; }
  int min_changes() const { return min_changes_; }
  const ProblemSolution& solution() const { return solution_; }
  void set_max_changes(int v) { max_changes_ = v; }
  void set_min_changes(int v) { min_changes_ = v; }
  void set_solution(const ProblemSolution& s) { solution_ = s; }
protected:
  int max_changes_;
  int min_changes_;
  ProblemSolution solution_;
};

class VnsCutUtil {
 public:
  static bool IsValidFinalXijMatrix(const vector<VnsCut>& cuts,
                                    const vector<vector<double> >& x);
  static bool CanFixCandidate(const vector<VnsCut>& cuts,
                              const vector<vector<short> >& fixed,
                              int fixing_machine, int fixing_task,
                              FixingSense sense);

  static int Delta(const VnsCut& cut, const vector<vector<double> >& x);
  static void Delta(
     const VnsCut& cut, const vector<vector<short> >& already_fixed,
     int fixing_machine, int fixing_task, FixingSense fixing_sense,
     int* already_changed, int* still_can_change);
};

#endif