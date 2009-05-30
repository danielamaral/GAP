#ifndef ELLIPSOIDALCUT_H
#define ELLIPSOIDALCUT_H

#include <map>
#include <vector>

#include "Globals.h"
#include "opt_row.h"
#include "ProblemSolution.h"

class EllipsoidalCut;

class EllipsoidalCutUtil {
 public:
  static bool IsValidFinalXijMatrix(const vector<EllipsoidalCut>& cuts,
                                    const vector<vector<double> >& x);

  static int Delta(const EllipsoidalCut& cut, const vector<vector<double> >& x);

  static int Intersection(const ProblemSolution& a, const ProblemSolution& b);
};

class EllipsoidalCut {
 public:
  EllipsoidalCut(const vector<ProblemSolution*>& x,
                 OPT_ROW::ROWSENSE constraint_sense, int F):
      k_(F), sense_(constraint_sense) {
    CalculateWeights(x);
    rhs_ = Globals::instance()->num_machines() * x.size() - k_;
  }
  OPT_ROW::ROWSENSE sense() const { return sense_; }
  int rhs() const { return rhs_; }
  int k() const { return k_; }
  int weight(int machine, int task) const {
    map<int, map<int, int> >::const_iterator it = assignment_weights_.find(task);
    if (it != assignment_weights_.end()) {
      map<int, int>::const_iterator counts = it->second.find(machine);
      if (counts != it->second.end()) {
        return counts->second;
      }
    }
    return 0;
  }
 protected:
  int alfa_;
  int k_;
  int rhs_;
  OPT_ROW::ROWSENSE sense_;
  map<int, map<int, int> > assignment_weights_;

  void CalculateWeights(const vector<ProblemSolution*>& x) {
    assignment_weights_.clear();
    // for each task
    for (int task = 0; task < Globals::instance()->n(); ++task) {
      // counts the weight of every machine assignment
      map<int, int>* machine_to_counts = &(assignment_weights_[task]);
      for (int sol = 0; sol < x.size(); ++sol) {
        (*machine_to_counts)[x[sol]->assignment(task)] += 1;
      }
    }
  }

 private:
  EllipsoidalCut();
};

#endif