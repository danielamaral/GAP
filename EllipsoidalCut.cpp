#include "EllipsoidalCut.h"

#include "logging.h"

/* static */ bool EllipsoidalCutUtil::IsValidFinalXijMatrix(
    const vector<EllipsoidalCut>& cuts, const vector<vector<double> >& x) {
  for (int i = 0; i < cuts.size(); ++i) {
    int lhs = EllipsoidalCutUtil::Delta(cuts[i], x);
    if (cuts[i].sense() == OPT_ROW::LESS && lhs > cuts[i].rhs()) {
      return false;
    } else if (cuts[i].sense() == OPT_ROW::EQUAL && lhs != cuts[i].rhs()) {
      return false;
    } else if (cuts[i].sense() == OPT_ROW::GREATER && lhs < cuts[i].rhs()) {
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
  for (int machine = 0; machine < Globals::instance()->num_machines(); ++machine) {
    for (int task = 0; task < Globals::instance()->num_tasks(); ++task) {
      double assignment = final_x[machine][task];
      CHECK(assignment < Globals::EPS() || assignment > 1.0 - Globals::EPS());
      if (assignment > 1.0 - Globals::EPS()) {
        lhs += cut.weight(machine, task);
      }
    }
  }
  return lhs;
}
