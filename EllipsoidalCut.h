#ifndef ELLIPSOIDALCUT_H
#define ELLIPSOIDALCUT_H

#include <vector>

#include "Globals.h"
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
  EllipsoidalCut(const ProblemSolution& s1, const ProblemSolution& s2, int k):
      k_(k), solution1_(s1), solution2_(s2) {
    alfa_ = EllipsoidalCutUtil::Intersection(s1, s2);
  }
  int alfa() const { return alfa_; }
  int k() const { return k_; }
  const ProblemSolution& solution1() const { return solution1_; }
  const ProblemSolution& solution2() const { return solution2_; }
 protected:
  int alfa_;
  int k_;
  ProblemSolution solution1_;
  ProblemSolution solution2_;
 private:
  EllipsoidalCut();
};

#endif