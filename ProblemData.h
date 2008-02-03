#pragma once

class ProblemData
{
public:
    ProblemData();
    ~ProblemData(void);

    /// Clears the data for reuse
    void clear();

    /// Returns the cost of assigning 'task' to 'machine'
    int cost(int machine, int task) const;
    /// Sets the cost of assigning 'task' to 'machine'
    void set_cost(int machine, int task, int c);

    /// Returns the resources consumed in 'machine' by assigning it 'task'
    int consume(int machine, int task) const;
    /// Sets the resources consumed in 'machine' by assigning it 'task'
    void set_consume(int machine, int task, int c);

    /// Returns the capacity of 'machine'
    int capacity(int machine) const;
    /// Sets the capacity of 'machine'
    void set_capacity(int machine, int c);
    
    /// Returns the number of machines (agents, etc)
    int m() const;
    /// Sets the number of machines
    void set_m(int x);

    /// Returns the number of tasks (jobs, etc)
    int n() const;
    /// Sets the number of tasks
    void set_n(int x);
private:
    int m_;
    int n_;

    static const int kMaxMachines = 80;
    static const int kMaxTasks = 1600;
    int cost_[kMaxMachines][kMaxTasks];
    int consume_[kMaxMachines][kMaxTasks];
    int capacity_[kMaxMachines];
};
