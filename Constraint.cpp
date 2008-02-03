#include "Constraint.h"
#include "HashUtil.h"
#include <stdio.h>

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
   if( (int)this->getType() < (int) cons.getType() )
      return true;
   else if( (int)this->getType() > (int) cons.getType() )
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

std::string Constraint::toString()
{
	std::string consName;
   char auxStr[100];

   	/*
	ToDo:
	*/

	return consName;
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
