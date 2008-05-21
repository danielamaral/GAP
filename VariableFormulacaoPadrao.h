#pragma once

#include <cstdio>
#include <map>
#include <hash_map>

//Variables
class VariableFormulacaoPadrao
{
    //Struct used by hash table
    friend class VariableFormulacaoPadraoHasher;

public:

    // All variable types
    enum VariableType
    {
	    V_ERROR = 0,
	    X_ij  // Task J assigned to machine I
    };

   //Constructors
   VariableFormulacaoPadrao();
   VariableFormulacaoPadrao(const VariableFormulacaoPadrao& orig);

   //Destructor
   virtual ~VariableFormulacaoPadrao();
   
    //==================================================
    // GET METHODS 
    //==================================================
    // Return variable type
    VariableType type() const {
        return type_;
    }
	// Return the machine
	int machine() const { 
        return machine_;
    }
	// Return the task
	int task() const {
        return task_;
    }
    double value() const {
        return value_;
    }
    double reducedcost() const {
        return reducedcost_;
    }

	
	//==================================================
   // SET METHODS 
   //==================================================
   // Reset variables values
	void reset();
	// Set variable type
	void set_type(VariableType t) {
        type_ = t;
    }
    void set_machine(int machine) {
        machine_ = machine;
    }
    void set_task(int task) {
        task_ = task;
    }
	// Set value
	void set_value(double value) {
        value_ = value;
    }
	// Set reduced cost
	void set_reducedcost(double reducedcost) {
        reducedcost_ = reducedcost;
    }

   //==================================================
   // OPERATORS 
   //==================================================
   //Assignment 
   VariableFormulacaoPadrao& operator=(const VariableFormulacaoPadrao& var);
   //Less 
   bool operator<(const VariableFormulacaoPadrao& var) const;
   //Equals 
   bool operator==(const VariableFormulacaoPadrao& var) const;

   //Variable name
   std::string ToString() const;

private:

	//tipo da variavel
    VariableType type_;

    // machine
    int machine_;

    // task
    int task_;

	double value_;
	double reducedcost_;
};


class VariableFormulacaoPadraoHasher : public stdext::hash_compare<VariableFormulacaoPadrao>
{
public:
   //Less operator
   bool operator()(const VariableFormulacaoPadrao& v1, const VariableFormulacaoPadrao& v2) const;

   //Hash value
   size_t operator()(const VariableFormulacaoPadrao& v) const;
};

class VariableFormulacaoPadraoPtrHasher : public stdext::hash_compare<VariableFormulacaoPadrao*>
{
public:
   //Less operator
   bool operator()(const VariableFormulacaoPadrao* v1, const VariableFormulacaoPadrao* v2) const;

   //Hash value
   size_t operator()(const VariableFormulacaoPadrao* v) const;
};

/**
 * Type definition for the hash object.
 */
typedef stdext::hash_map<VariableFormulacaoPadrao, int, VariableFormulacaoPadraoHasher> VariableFormulacaoPadraoHash;
typedef std::pair<VariableFormulacaoPadrao, int> VariableFormulacaoPadraoIntPair;
