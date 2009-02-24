#pragma once

#include <cstdio>
#include <hash_map>
#include <string>

using namespace std;
/**
 * Class which defines a contraint in the LP.
 */
class ConstraintGeracaoColunas
{
   //Struct used by hash table
   friend class ConstraintGeracaoColunasHasher;

public:

	/** All constraint types */
	enum ConstraintType
	{
		C_ERROR = 0,
    C_MACHINE_CAPACITY,
    C_ONE_MACHINE_PER_TASK,
    C_MAX_ASSIGNMENT_CHANGES,
		C_MIN_ASSIGNMENT_CHANGES,
		C_MAX_ASSIGNMENT_CHANGES_ELLIPSOIDAL,
		C_INTEGER_STRONG_CUTTING_PLANE
	};

	/** Default constructor. */
	ConstraintGeracaoColunas();

	/** Copy constructor. */
	ConstraintGeracaoColunas(const ConstraintGeracaoColunas& cons);

	/** Destructor. */
	~ConstraintGeracaoColunas();

	/** Assign operator. */
	ConstraintGeracaoColunas& operator= (const ConstraintGeracaoColunas& cons);

	/** Less operator. */
	bool operator< (const ConstraintGeracaoColunas& cons) const;

	/** Equals operator. */
	bool operator== (const ConstraintGeracaoColunas& cons) const;

	/**
	 * Returns a string containing an unique name for the constraint.
	 * @return a string containing an unique name for the constraint.
	 */
	std::string ToString() const;

   //==================================================
   // GET METHODS 
   //==================================================
    
   // type
	ConstraintType type() const	{
        return type_;
    }

	// machine
	int machine() const {
        return machine_;
    }

    // task
    int task() const {
        return task_;
    }

    // dual value
    double dualvalue() const {
        return dualvalue_;
    }

   //==================================================
   // SET METHODS 
   //==================================================

    // reset variables values
    void reset();

    // set constraint type
    void set_type(ConstraintType t)	{
      type_ = t;
    }

    // set machine
	  void set_machine(int machine) {
        machine_ = machine;
    }

    // set task
    void set_task(int task) {
      task_ = task;
    }
	
    // set dual value
    void set_dualvalue(double dualvalue) {
      dualvalue_ = dualvalue;
    }
  
private:
	ConstraintType type_;
  int machine_;
  int task_;	
	double dualvalue_;
};

/**
 * Defines the operations needed by the hash object.
 */
class ConstraintGeracaoColunasHasher :
  public stdext::hash_compare<ConstraintGeracaoColunas> {
public:
	/**
	 * Applies the hash function on a Constraint object.
	 * @param cons The constraint.
	 * @return The hash value for the constraint.
	 */
	size_t operator() (const ConstraintGeracaoColunas& cons) const;

	/**
	 * Defines an order rule for two Constraint objects.
	 * @param cons1 The first constraint to be compared.
	 * @param cons2 The second constraint to be compared.
	 * @return True if const1 comes before cons2, false otherwise.
	 */
	bool operator() (const ConstraintGeracaoColunas& cons1,
                   const ConstraintGeracaoColunas& cons2) const;
};

/**
 * Type definition for the hash object.
 */
typedef stdext::hash_map<
  ConstraintGeracaoColunas, int, ConstraintGeracaoColunasHasher>
ConstraintGeracaoColunasHash;