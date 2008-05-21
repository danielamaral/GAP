#pragma once

#include <iostream>
#include <vector>

class ProblemData;

//Stores output data
class ProblemSolution
{
	friend std::ostream& operator << (std::ostream& out, ProblemSolution& solution );
public:
   //Constructor
   ProblemSolution(const ProblemData* pd);

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
   /// Returns the number of machines
   int m() const;
   /// To which machine was 'task' assigned in the solution?
   int assignment(int task) const;
   /// How much of the machine's capacity is being used
   int used(int machine) const;
   /// The final cost of the solution
   int cost() const;
   /// Clears the solution to its unassigned state
   void Reset();
   /// The number of different assignments between 2 solutions
   int Distance(const ProblemSolution& sol) const;
   /// Is this a valid assignment?
   bool IsValid() const;
   /// Is this a valid exchange of task assignments?
   bool IsValidExchange(int task1, int task2) const;
   void Exchange(int task1, int task2);

   bool operator==(const ProblemSolution& sol);
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

std::ostream& operator << (std::ostream& out, ProblemSolution& solution );