#include "SolutionSet.h"

#include "Globals.h"
#include "logging.h"

FixedSizeSolutionSet::FixedSizeSolutionSet(int size) {
  mem_buffer_.resize(size);
  max_size_ = size;
}

FixedSizeSolutionSet::~FixedSizeSolutionSet() {
  while (!sol_map_.empty())
    RemoveWorstSolution();
}

bool FixedSizeSolutionSet::AddSolution(const ProblemSolution& s) {
  if (sol_map_.size() < max_size_) {
    AddSolution(s, sol_map_.size());
    return true;
  }

  // Set is full, add if not present and better than worst cost.
  SolutionMap::iterator it = sol_map_.find(const_cast<ProblemSolution*>(&s));
  if (it == sol_map_.end() && heap_.top()->cost() > s.cost()) {
    int index = RemoveWorstSolution();
    // Adds the new solution.
    AddSolution(s, index);
    return true;
  }

  return false;
}

int FixedSizeSolutionSet::RemoveWorstSolution() {
  // Remove the worst solution.
  SolutionMap::iterator it = sol_map_.find(heap_.top());
  int buffer_index = it->second;
  heap_.pop();
  delete it->first;
  sol_map_.erase(it);
  return buffer_index;
}

void FixedSizeSolutionSet::AddSolution(const ProblemSolution& s, int pos) {
  mem_buffer_[pos] = new ProblemSolution(s);
  sol_map_.insert(make_pair(mem_buffer_[pos], pos));
  heap_.push(mem_buffer_[pos]);
}

const ProblemSolution& FixedSizeSolutionSet::GetSolution(int i) const {
  CHECK_LT(i, sol_map_.size());
  return *mem_buffer_[i];
}

int FixedSizeSolutionSet::GetSize() const {
  return sol_map_.size();
}