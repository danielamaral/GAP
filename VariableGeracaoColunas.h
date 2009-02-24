#pragma once

#include <cstdio>
#include <map>
#include <hash_map>

//Variables
class VariableGeracaoColunas {
    //Struct used by hash table
    friend class VariableGeracaoColunasHasher;

public:

    // All variable types
    enum VariableType {
	    V_ERROR = 0,
	    X_ij  // Task J assigned to machine I
    };

   //Constructors
   VariableGeracaoColunas();
   VariableGeracaoColunas(const VariableGeracaoColunas& orig);

   //Destructor
   virtual ~VariableGeracaoColunas();
   
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
   VariableGeracaoColunas& operator=(const VariableGeracaoColunas& var);
   //Less 
   bool operator<(const VariableGeracaoColunas& var) const;
   //Equals 
   bool operator==(const VariableGeracaoColunas& var) const;

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


class VariableGeracaoColunasHasher :
  public stdext::hash_compare<VariableGeracaoColunas> {
public:
   //Less operator
   bool operator()(const VariableGeracaoColunas& v1,
                   const VariableGeracaoColunas& v2) const;

   //Hash value
   size_t operator()(const VariableGeracaoColunas& v) const;
};

class VariableGeracaoColunasPtrHasher :
  public stdext::hash_compare<VariableGeracaoColunas*> {
public:
   //Less operator
   bool operator()(const VariableGeracaoColunas* v1,
                   const VariableGeracaoColunas* v2) const;

   //Hash value
   size_t operator()(const VariableGeracaoColunas* v) const;
};

/**
 * Type definition for the hash object.
 */
typedef stdext::hash_map<
  VariableGeracaoColunas, int, VariableGeracaoColunasHasher>
VariableGeracaoColunasHash;

typedef std::pair<VariableGeracaoColunas, int> VariableGeracaoColunasIntPair;