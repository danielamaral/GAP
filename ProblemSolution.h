#pragma once

#include <iostream>
#include <vector>

using namespace std;

class ProblemData;

//Stores output data
class ProblemSolution
{
	friend std::ostream& operator << (std::ostream& out, ProblemSolution& solution );
public:
   //Constructor
   ProblemSolution(const ProblemData* pd);
   ProblemSolution(const ProblemSolution& p);

   //Destructor
   ~ProblemSolution() {}

   //==================================================
   // SET METHODS 
   //==================================================

   /**
   ToDo:
   All set methods of the private attributes should be defined here
   */
   /// Set to which machine 'task' is assigned in the solution
   void set_assignment(int task, int machine);

   //==================================================
   // GET METHODS 
   //==================================================

   /**
   ToDo:
   All get methods of the private attributes should be defined here
   */
   /// Returns the number of tasks
   int n() const;
   int num_tasks() const;
   /// Returns the number of machines
   int m() const;
   int num_machines() const;
   /// To which machine was 'task' assigned in the solution?
   int assignment(int task) const;
   /// How much of the machine's capacity is being used
   int used(int machine) const;
   /// The final cost of the solution
   int cost() const;
   void set_cost(int cost);
   /// The cost of the current assignment of tasks to machine 'mac'. 
   int AssignmentCost(int mac) const;
   /// Clears the solution to its unassigned state
   void Clear();
   /// A boolean vector with the tasks assigned to machine 'mac'
   void GetMachineAssignments(int mac, vector<bool>* assign) const;
   /// A boolean vector with the tasks assigned to machine 'mac'
   void GetMachineAssignments(int mac, vector<short>* tasks) const;
   /// The number of different assignments between 2 solutions
   int Distance(const ProblemSolution& sol) const;
   /// Is this a valid assignment?
   bool IsValid() const;
   /// Is this a valid exchange of task assignments?
   bool IsValidExchange(int task1, int task2) const;
   void Exchange(int task1, int task2);

   bool operator==(const ProblemSolution& sol) const;
private:

	/**
	ToDo:
	All objects that define the problem output should be declared here
	**/
    std::vector<int> assignment_;
    std::vector<int> used_;
    int cost_;
    const ProblemData* pd_;
};

struct CompareSolutionPtrsByAssignment {
  bool operator()(ProblemSolution* a, ProblemSolution* b) const {
    for (int i = 0; i < a->n(); ++i) {
      if (a->assignment(i) < b->assignment(i))
        return true;
      if (a->assignment(i) > b->assignment(i))
        return false;
    }
    return false;
  }
};

struct CompareSolutionPtrsByCost {
  bool operator()(const ProblemSolution* a, const ProblemSolution* b) const {
    return a->cost() < b->cost();
  }
};


std::ostream& operator << (std::ostream& out, ProblemSolution& solution );