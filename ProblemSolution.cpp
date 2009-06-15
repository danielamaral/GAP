#include "ProblemSolution.h"

#include <cassert>
#include "Globals.h"
#include "logging.h"
#include "ProblemData.h"

using namespace std;

ProblemSolution::ProblemSolution(const ProblemData* pd) : pd_(pd)
{
	assignment_.resize(pd_->n(), -1);
  used_.resize(pd_->m(), 0);
	Clear();
}

ProblemSolution::ProblemSolution(const ProblemSolution& p) {
  pd_ = Globals::instance();
  assignment_.resize(p.n(), -1);
  used_.resize(p.m(), 0);
	Clear();

  if (p.assignment(0) >= 0)
    for (int i = 0; i < p.n(); ++i)
      this->set_assignment(i, p.assignment(i));
}

void ProblemSolution::set_assignment(int task, int machine) {
    assert(task < pd_->n());
    assert(machine < pd_->m());

    if (assignment_[task] >= 0) {
        int old_machine = assignment_[task];
        cost_ -= pd_->cost(old_machine, task);
        used_[old_machine] -= pd_->consume(old_machine, task);
    }
    assignment_[task] = machine;
    used_[machine] += pd_->consume(machine, task);
    cost_ += pd_->cost(machine, task);
}

int ProblemSolution::assignment(int task) const {
    assert(task < pd_->n());
    return assignment_[task];
}

int ProblemSolution::used(int machine) const {
    assert(machine < pd_->m());
    return used_[machine];
}

int ProblemSolution::n() const {
	return pd_->n();
}

int ProblemSolution::num_tasks() const {
	return this->n();
}

int ProblemSolution::m() const {
	return pd_->m();
}

int ProblemSolution::num_machines() const {
	return this->m();
}

int ProblemSolution::cost() const {
    return cost_;
}

void ProblemSolution::set_cost(int c) {
  cost_ = c;
}

int ProblemSolution::Distance(const ProblemSolution& sol) const {
	int d = 0;
	for (int i = 0; i < n(); ++i)
		if (assignment(i) != sol.assignment(i))
			d++;
	return d;
}

void ProblemSolution::Clear() {
    fill(assignment_.begin(), assignment_.end(), -1);
    fill(used_.begin(), used_.end(), 0);
    cost_ = 0;
}

void ProblemSolution::GetMachineAssignments(int mac,
                                            vector<bool>* assign) const {
  assign->reserve(this->num_tasks());
  for (int i = 0; i < this->num_tasks(); ++i)
    (*assign)[i] = this->assignment(i) == mac;
}

void ProblemSolution::GetMachineAssignments(int mac,
                                            vector<short>* tasks) const {
  tasks->clear();
  for (int i = 0; i < this->num_tasks(); ++i)
    if (this->assignment(i) == mac)
      tasks->push_back(i);
}

int ProblemSolution::AssignmentCost(int mac) const {
  int ret = 0;
  for (int i = 0; i < this->num_tasks(); ++i)
    if (this->assignment(i) == mac)
      ret += pd_->cost(mac, i);
  return ret;
}

bool ProblemSolution::IsValid() const {
    for (int i = 0; i < pd_->n(); ++i) {
        if (assignment_[i] < 0) {
            std::cout << "Assignment[" << i << "] = " << assignment_[i] << std::endl;
            return false;
        }
    }
    for (int i = 0; i < pd_->m(); ++i) {
        if (used_[i] > pd_->capacity(i)) {
            std::cout << "Used[" << i << "] = " << used_[i] << " (cap: " << pd_->capacity(i) << ")" << std::endl;
            return false;
        }
    }
    return true;
}

bool ProblemSolution::IsValidExchange(int task1, int task2) const {
	if (assignment(task1) == assignment(task2))
		return true;
	int mach1 = assignment(task1);
	int mach2 = assignment(task2);
	bool machine_one_fits = (pd_->capacity(mach1) >=
		used(mach1) - pd_->consume(mach1, task1) + pd_->consume(mach1, task2));
	bool machine_two_fits = (pd_->capacity(mach2) >=
		used(mach2) - pd_->consume(mach2, task2) + pd_->consume(mach2, task1));
	return machine_one_fits && machine_two_fits;
}

void ProblemSolution::Exchange(int task1, int task2) {
	CHECK(IsValidExchange(task1, task2));
	int mach1 = assignment(task1);
	int mach2 = assignment(task2);
	used_[mach1] = used_[mach1] - pd_->consume(mach1, task1) + pd_->consume(mach1, task2);
	used_[mach2] = used_[mach2] - pd_->consume(mach2, task2) + pd_->consume(mach2, task1);
	cost_ = cost_ - pd_->cost(mach1, task1) - pd_->cost(mach2, task2) + pd_->cost(mach1, task2) + pd_->cost(mach2, task1);
	assignment_[task1] = mach2;
	assignment_[task2] = mach1;
}

bool ProblemSolution::operator ==(const ProblemSolution& sol) const {
    return assignment_ == sol.assignment_;
}

//==================================================
//Write solution XML
//==================================================
std::ostream& operator << (std::ostream& out, ProblemSolution& solution )
{
	/**
	ToDo:
	The XML that describes the output should be written here
	**/
	for (int i = 0; i < solution.n(); ++i) {
		if (i > 0) {
			out << " ";
		}
		out << solution.assignment(i);
	}
    return out;
}