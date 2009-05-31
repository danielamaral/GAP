#ifndef ELLIPSOIDALCUT_H
#define ELLIPSOIDALCUT_H

#include <map>
#include <queue>
#include <vector>

#include "ProblemSolution.h"

class SolutionSet
{
public:
  SolutionSet() {};
  ~SolutionSet(void) {};
  virtual bool AddSolution(const ProblemSolution& s) = 0;
  virtual const ProblemSolution& GetSolution(int i) const = 0;
  virtual int size() const = 0;
};

class FixedSizeSolutionSet : public SolutionSet {
public:
  FixedSizeSolutionSet(int size);
  ~FixedSizeSolutionSet();
  virtual bool AddSolution(const ProblemSolution& s);
  virtual const ProblemSolution& GetSolution(int i) const;
  virtual const ProblemSolution* GetSolutionPtr(int i) const;
  virtual int size() const;
protected:
  vector<ProblemSolution*> mem_buffer_;
  priority_queue<ProblemSolution*, vector<ProblemSolution*>,
                 CompareSolutionPtrsByCost> heap_;
  typedef map<ProblemSolution*, int, CompareSolutionPtrsByAssignment> SolutionMap;
  SolutionMap sol_map_;

  int max_size_;

  void AddSolution(const ProblemSolution& s, int pos);
  int RemoveWorstSolution();
};

#endif