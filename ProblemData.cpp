#include "ProblemData.h"

#include "logging.h"

ProblemData::ProblemData() : m_(0), n_(0), minimum_assignment_cost_(0)
{
}


ProblemData::~ProblemData(void)
{
}

void ProblemData::clear() {
    m_ = n_ = 0;
    minimum_assignment_cost_ = 0;
}

void ProblemData::set_m(int x) {
    DCHECK(x <= kMaxMachines);
    m_ = x;
}

void ProblemData::set_n(int x) {
    DCHECK(x <= kMaxTasks);
    n_ = x;
}

int ProblemData::m() const {
    return m_;
}

int ProblemData::num_machines() const {
    return m();
}

int ProblemData::n() const {
    return n_;
}

int ProblemData::num_tasks() const {
    return n();
}

void ProblemData::set_cost(int machine, int task, int c) {
    DCHECK(machine < m_);
    DCHECK(task < n_);
    DCHECK(c >= 0);
    cost_[machine][task] = c;
    minimum_assignment_cost_ = min<int>(minimum_assignment_cost_, c);
}

int ProblemData::cost(int machine, int task) const {
    DCHECK(machine < m_);
    DCHECK(task < n_);
    return cost_[machine][task];
}

void ProblemData::set_consume(int machine, int task, int c) {
    DCHECK(machine < m_);
    DCHECK(task < n_);
    DCHECK(c >= 0);
    consume_[machine][task] = c;
}

int ProblemData::consume(int machine, int task) const {
    DCHECK(machine < m_);
    DCHECK(task < n_);
    return consume_[machine][task];
}

void ProblemData::set_capacity(int machine, int c) {
    DCHECK(machine < m_);
    DCHECK(c >= 0);
    capacity_[machine] = c;
}

int ProblemData::capacity(int machine) const {
    DCHECK(machine < m_);
    return capacity_[machine];
}

int ProblemData::optimal() const {
    return optimal_;
}
void ProblemData::set_optimal(int o) {
    optimal_ = o;
}

int ProblemData::AssignmentCost(int machine, const vector<int>& assignment) const {
  int cost = 0;
  for (int t = 0; t < this->num_tasks(); ++t) {
    if (assignment[t] == 1)
      cost += this->cost(machine, t);
  }
  return cost;
}

int ProblemData::AssignmentConsume(int machine, const vector<int>& assignment) const {
  int consume = 0;
  for (int t = 0; t < this->num_tasks(); ++t) {
    if (assignment[t] == 1)
      consume += this->consume(machine, t);
  }
  return consume;
}

int ProblemData::MinimumAssignmentCost() const {
  return minimum_assignment_cost_;
}

const int* ProblemData::GetConsumeVector(int machine) const {
  return consume_[machine];
}