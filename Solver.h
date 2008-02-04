#pragma once

class ProblemData;
class ProblemSolution;

/**
* Defines a generic solver.
*/
class Solver
{
public:
   /**
   * Default Constructor.
   * @param aProblemData The problem's input data.
   */
   Solver(ProblemData *aProblemData);

   /** Destructor. */
   ~Solver();

   /**
   * Solves the problem.
   * @return The solution status.
   */
   virtual int solve() = 0;

   /**
   * Processes the variable values and populates the output class.
   * @param ps A reference to the class to be populated.
   */
	virtual void generate_solution(ProblemSolution *ps) {}

protected:
   /** A reference to the problem's input data. */
   ProblemData* problem_data_;
};