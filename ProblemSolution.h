#pragma once

#include <iostream>

//Stores output data
class ProblemSolution
{
	friend std::ostream& operator << (std::ostream& out, ProblemSolution& solution );
public:
   //Constructor
   ProblemSolution();

   //Destructor
   ~ProblemSolution() {}

   //==================================================
   // SET METHODS 
   //==================================================

   /**
   ToDo:
   All set methods of the private attributes should be defined here
   */

   //==================================================
   // GET METHODS 
   //==================================================

   /**
   ToDo:
   All get methods of the private attributes should be defined here
   */

private:

	/**
	ToDo:
	All objects that define the problem output should be declared here
	**/

};

std::ostream& operator << (std::ostream& out, ProblemSolution& solution );