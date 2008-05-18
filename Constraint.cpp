#include "Constraint.h"

#include <cstdio>
#include <sstream>

Constraint::Constraint()
{
	reset();

	/*
	ToDo:
	Attributes should be initiated
	*/
}

Constraint::Constraint(const Constraint &cons)
{
	*this = cons;
}

Constraint::~Constraint()
{
   reset();
}

Constraint& Constraint::operator= (const Constraint& cons)
{   
  /**
  ToDo:
  */
   
   return *this;
}

bool Constraint::operator< (const Constraint& cons) const
{
   if( (int)this->type() < (int) cons.type() )
      return true;
   else if( (int)this->type() > (int) cons.type() )
      return false;

	/*
	ToDo:
	*/

   return false;
}

bool Constraint::operator== (const Constraint& cons) const
{
	return (!(*this < cons) && !(cons < *this));
}

void Constraint::reset()
{
	/*
	ToDo:
	All pointers that define a constraint should be addressed to NULL
	*/
}

std::string Constraint::ToString()
{
    std::stringstream ss;

   	/*
	ToDo:
	*/

	return ss.str();
}

size_t ConstraintHasher::operator() (const Constraint& cons) const
{
   unsigned int sum = 0;

   /**
   ToDo:
     All pointers different from NULL must be considered in the hash function
   **/

   return sum;
}

bool ConstraintHasher::operator() (const Constraint& cons1, const Constraint& cons2) const
{
	return (cons1 < cons2);
}
