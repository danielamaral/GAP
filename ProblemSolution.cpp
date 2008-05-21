#include "ProblemSolution.h"

#include <cassert>
#include "ProblemData.h"

using namespace std;

ProblemSolution::ProblemSolution(const ProblemData* pd) : pd_(pd)
{
	assignment_.resize(pd->n(), -1);
    used_.resize(pd->m(), 0);
	Reset();
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

int ProblemSolution::m() const {
	return pd_->m();
}

int ProblemSolution::cost() const {
    return cost_;
}

int ProblemSolution::Distance(const ProblemSolution& sol) const {
	int d = 0;
	for (int i = 0; i < n(); ++i)
		if (assignment(i) != sol.assignment(i))
			d++;
	return d;
}

void ProblemSolution::Reset() {
    fill(assignment_.begin(), assignment_.end(), -1);
    fill(used_.begin(), used_.end(), 0);
    cost_ = 0;
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
	if (!IsValidExchange(task1, task2))
		return;
	int mach1 = assignment(task1);
	int mach2 = assignment(task2);
	used_[mach1] = used_[mach1] - pd_->consume(mach1, task1) + pd_->consume(mach1, task2);
	used_[mach2] = used_[mach2] - pd_->consume(mach2, task2) + pd_->consume(mach2, task1);
	cost_ = cost_ - pd_->cost(mach1, task1) - pd_->cost(mach2, task2) + pd_->cost(mach1, task2) + pd_->cost(mach2, task1);
	assignment_[task1] = mach2;
	assignment_[task2] = mach1;
}

bool ProblemSolution::operator ==(const ProblemSolution& sol) {
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