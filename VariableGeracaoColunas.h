#pragma once

#include <cstdio>

#include <hash_map>
#include <map>
#include <vector>

using namespace std;

class Column {
public:
  Column(int machine, int index, const vector<int>& tasks) :
      machine_(machine),
      index_(index),
      tasks_(tasks) {
  }

  Column(const Column& c) {
    this->machine_ = c.machine();
    this->tasks_ = c.tasks();
    this->index_ = c.index();
  }

  Column() {
    Clear();
  }

  void Clear() {
    this->machine_ = -1;
    this->index_ = -1;
    this->tasks_.clear();
  }

  void set_index(int i) { index_ = i; }
  int index() const { return index_; }
  int machine() const { return machine_; }
  const vector<int>& tasks() const { return tasks_; }

private:
  vector<int> tasks_;
  int machine_;
  int index_;
};

//Variables
class VariableGeracaoColunas {
    //Struct used by hash table
    friend class VariableGeracaoColunasHasher;

public:

    // All variable types
    enum VariableType {
	    V_ERROR = 0,
      Z_k,  // Positive identity, for stabilization
      W_k,  // Negative identity, for stabilization
      COL  // A generated column
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
    // Returns the column (only for variables of type COL)
    const Column& column() const {
      return column_;
    }

    double value() const {
      return value_;
    }

    double reducedcost() const {
      return reducedcost_;
    }

    double cost() const {
      return cost_;
    }
	
    //==================================================
    // SET METHODS 
    //==================================================
    // Reset variables values
	  
    void Clear();

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

    void set_column(const Column& column) {
      this->column_ = column;
    }

    void set_column_index(int index) {
      this->column_.set_index(index);
    }

	  void set_value(double value) {
        value_ = value;
    }

    void set_cost(double cost) {
      cost_ = cost;
    }

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
  VariableType type_;
  int machine_;
  int task_;
  Column column_;

	double value_;
  double cost_;
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