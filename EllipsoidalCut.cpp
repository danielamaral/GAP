#include "EllipsoidalCut.h"

#include "logging.h"

/* static */ bool EllipsoidalCutUtil::IsValidFinalXijMatrix(const vector<EllipsoidalCut>& cuts,
                                                    const vector<vector<double> >& x) {
  for (int i = 0; i < cuts.size(); ++i) {
    int delta = EllipsoidalCutUtil::Delta(cuts[i], x);
    if (delta < cuts[i].solution1().n() + cuts[i].alfa() - cuts[i].k()) {
      return false;
    }
  }
  return true;
}

/* static */ int EllipsoidalCutUtil::Intersection(
    const ProblemSolution& a,
    const ProblemSolution& b) {
  int intersection = 0;
  for (int i = 0; i < a.n(); ++i)
    if (a.assignment(i) == b.assignment(i))
      ++intersection;
  return intersection;
}

/* static */ int EllipsoidalCutUtil::Delta(
    const EllipsoidalCut& cut,
    const vector<vector<double> >& final_x) {
  int lhs = 0;
  for (int i = 0; i < cut.solution1().n(); ++i) {
    double assignment = final_x[cut.solution1().assignment(i)][i];
    CHECK(assignment < Globals::EPS() || assignment > 1.0 - Globals::EPS());
    if (assignment > 1.0 - Globals::EPS()) {
      ++lhs;
    }
    assignment = final_x[cut.solution2().assignment(i)][i];
    CHECK(assignment < Globals::EPS() || assignment > 1.0 - Globals::EPS());
    if (assignment > 1.0 - Globals::EPS()) {
      ++lhs;
    }
  }
  return lhs;
}
