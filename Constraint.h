#ifndef CONSTRAINT_H_
#define CONSTRAINT_H_

#include <hash_map>

/** All constraint types */
enum ConstraintType
{
   C_ERROR = 0,

	/*
	ToDo:
	All constraints should be defined here
	*/
};

/**
 * Class which defines a contraint in the LP.
 */
class Constraint
{
   //Struct used by hash table
   friend class ConstraintHasher;

public:
	/** Default constructor. */
	Constraint();

	/** Copy constructor. */
	Constraint(const Constraint& cons);

	/** Destructor. */
	~Constraint();

	/** Assign operator. */
	Constraint& operator= (const Constraint& cons);

	/** Less operator. */
	bool operator< (const Constraint& cons) const;

	/** Equals operator. */
	bool operator== (const Constraint& cons) const;

	/**
	 * Returns a string containing an unique name for the constraint.
	 * @return a string containing an unique name for the constraint.
	 */
	std::string toString();

   //==================================================
   // GET METHODS 
   //==================================================
   //Return constraint type
   ConstraintType getType() const            { return type; }

   /*
   ToDo:
   All get methods of the private attributes should be defined here
   */

   //==================================================
   // SET METHODS 
   //==================================================
   // Reset variables values
   void reset();
   // Set constraint type
   void setType(ConstraintType t)               { type = t; }

   /*
   ToDo:
   All set methods of the private attributes should be defined here
   */
  
private:

	/** Attribute which defines the constraint type of the instance. */
	ConstraintType type;

	/**
	ToDo:
	All atributes which define a constraint should be declared here
	**/

};

/**
 * Defines the operations needed by the hash object.
 */
class ConstraintHasher : public stdext::hash_compare<Constraint>
{
public:
	/**
	 * Applies the hash function on a Constraint object.
	 * @param cons The constraint.
	 * @return The hash value for the constraint.
	 */
	size_t operator() (const Constraint& cons) const;

	/**
	 * Defines an order rule for two Constraint objects.
	 * @param cons1 The first constraint to be compared.
	 * @param cons2 The second constraint to be compared.
	 * @return True if const1 comes before cons2, false otherwise.
	 */
	bool operator() (const Constraint& cons1, const Constraint& cons2) const;
};

/**
 * Type definition for the hash object.
 */
typedef stdext::hash_map<Constraint, int, ConstraintHasher> ConstraintHash;

#endif