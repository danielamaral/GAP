#include "ProblemData.h"

#include <cassert>

ProblemData::ProblemData() : m_(0), n_(0)
{
}


ProblemData::~ProblemData(void)
{
}

void ProblemData::clear() {
    m_ = n_ = 0;
}

void ProblemData::set_m(int x) {
    assert(x < kMaxMachines);
    m_ = x;
}

void ProblemData::set_n(int x) {
    assert(x < kMaxTasks);
    n_ = x;
}

int ProblemData::m() const {
    return m_;
}

int ProblemData::n() const {
    return n_;
}

void ProblemData::set_cost(int machine, int task, int c) {
    assert(machine < m_);
    assert(task < n_);
    assert(c >= 0);
    cost_[machine][task] = c;
}

int ProblemData::cost(int machine, int task) const {
    assert(machine < m_);
    assert(task < n_);
    return cost_[machine][task];
}

void ProblemData::set_consume(int machine, int task, int c) {
    assert(machine < m_);
    assert(task < n_);
    assert(c >= 0);
    consume_[machine][task] = c;
}

int ProblemData::consume(int machine, int task) const {
    assert(machine < m_);
    assert(task < n_);
    return consume_[machine][task];
}

void ProblemData::set_capacity(int machine, int c) {
    assert(machine < m_);
    assert(c >= 0);
    capacity_[machine] = c;
}

int ProblemData::capacity(int machine) const {
    assert(machine < m_);
    return capacity_[machine];
}