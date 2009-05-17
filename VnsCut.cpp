#include "VnsCut.h"

#include "logging.h"

/* static */ bool VnsCutUtil::IsValidFinalXijMatrix(const vector<VnsCut>& cuts,
                                                    const vector<vector<double> >& x) {
  for (int i = 0; i < cuts.size(); ++i) {
    int delta = VnsCutUtil::Delta(cuts[i], x);
    if (cuts[i].max_changes() >= 0 && delta > cuts[i].max_changes())
      return false;
    if (cuts[i].min_changes() >= 0 && delta < cuts[i].min_changes())
      return false;
  }
  return true;
}

/* static */ bool VnsCutUtil::CanFixCandidate(const vector<VnsCut>& cuts,
                                              const vector<vector<short> >& fixed,
                                              int fixing_machine, int fixing_task,
                                              FixingSense sense) {
  int already_changed;
  int still_can_change;
  for (int i = 0; i < cuts.size(); ++i) {
    VnsCutUtil::Delta(cuts[i], fixed, fixing_machine, fixing_task,
                      sense, &already_changed, &still_can_change);
    //VLOG(1) << "VnsCutUtil: already_changed: " << already_changed << ", still_can_change: "
    //        << still_can_change;
    if (cuts[i].max_changes() > 0 && already_changed > cuts[i].max_changes())
      return false;
    if (cuts[i].min_changes() > 0 && already_changed + still_can_change < cuts[i].min_changes())
      return false;
  }
  return true;
}

/* static */ void VnsCutUtil::Delta(
     const VnsCut& cut, const vector<vector<short> >& already_fixed,
     int fixing_machine, int fixing_task, FixingSense fixing_sense,
     int* already_changed, int* still_can_change) {
  *already_changed = 0;
  *still_can_change = 0;
  for (int task = 0; task < cut.solution().n(); ++task) {
    if (task == fixing_task) {
      CHECK_EQ(-1, already_fixed[fixing_machine][fixing_task]);
      int machine_in_cut = cut.solution().assignment(fixing_task);
      if ((fixing_sense == FIX_ON_ONE && fixing_machine != machine_in_cut) ||
           (fixing_sense == FIX_ON_ZERO && fixing_machine == machine_in_cut)) {
        *already_changed += 1;
      } else if (fixing_sense == FIX_ON_ZERO && fixing_machine != machine_in_cut) {
        *still_can_change += 1;
      }
    } else {
      if (already_fixed[cut.solution().assignment(task)][task] == 0) {
        *already_changed += 1;
      } else if (already_fixed[cut.solution().assignment(task)][task] < 0) {
        *still_can_change += 1;
      }
    }
  }
}

/* static */ int VnsCutUtil::Delta(const VnsCut& cut, const vector<vector<double> >& x) {
  int delta = 0;
  for (int i = 0; i < cut.solution().n(); ++i) {
    const double& assignment = x[cut.solution().assignment(i)][i];
    if (assignment < Globals::EPS()) {  // if the assignment is fixed to zero
      ++delta;
    }
    // If the assignment is fixed to one, it maches the cut's solution. If it's
    // fixed to a fractionary value in-between (0, 1), it shouldn't be analyzed.
  }
  return delta;
}
