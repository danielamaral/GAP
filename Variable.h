#pragma once

#include <map>
#include <hash_map>

// All variable types
enum VariableType
{
	V_ERROR = 0,
};

//Variables
class Variable 
{
   //Struct used by hash table
   friend class VariableHasher;

public:
   //Constructors
   Variable();
   Variable(const Variable& orig);

   //Destructor
   virtual ~Variable();
   
   //==================================================
   // GET METHODS 
   //==================================================
   //Return variable type
   VariableType type() const {
       return type_;
   }

    // Return value
   double value() const {
       return value_;
   }

   /*
   ToDo:
   All get methods of the private attributes should be defined here
   */
   
   //==================================================
   // SET METHODS 
   //==================================================
   // Reset variables values
   void reset();

   // Set variable type
   void set_type(VariableType t) {
       type_ = t;
   }

   // Set value
   void set_value(double v) {
       value_ = v;
   }

   /*
   ToDo:
   All set methods of the private attributes should be defined here
   */

   //==================================================
   // OPERATORS 
   //==================================================
   //Assignment 
   Variable& operator=(const Variable& var);
   //Less 
   bool operator<(const Variable& var) const;
   //Equals 
   bool operator==(const Variable& var) const;

   //Variable name
   std::string toString();

private:
   VariableType type_;
   double value_;
   
   /* ToDo:
      All attributes that define a variable should be declared here
   */
 
};


class VariableHasher : public stdext::hash_compare<Variable>
{
public:
   //Less operator
   bool operator()(const Variable& v1, const Variable& v2) const;

   //Hash value
   size_t operator()(const Variable& v) const;
};

/**
 * Type definition for the hash object.
 */
typedef stdext::hash_map<Variable, int, VariableHasher> VariableHash;
