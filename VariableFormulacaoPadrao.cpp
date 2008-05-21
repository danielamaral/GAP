#include "VariableFormulacaoPadrao.h"

#include <sstream>
#include <functional>

VariableFormulacaoPadrao::VariableFormulacaoPadrao()
{
	reset();
}

VariableFormulacaoPadrao::VariableFormulacaoPadrao(const VariableFormulacaoPadrao& var)
{
    *this = var;
}

void VariableFormulacaoPadrao::reset()
{
    type_ = V_ERROR;
	
    // machine
	machine_ = -1;

	// task
	task_ = -1;

	value_ = 0.0;
	reducedcost_ = 0.0;
}

VariableFormulacaoPadrao::~VariableFormulacaoPadrao() {
	reset();
}

VariableFormulacaoPadrao& VariableFormulacaoPadrao::operator=(const VariableFormulacaoPadrao& var) {
	type_ = var.type();
    machine_ = var.machine();
    task_ = var.task();
    value_ = var.value();
    reducedcost_ = var.reducedcost();

	return *this;
}

bool VariableFormulacaoPadrao::operator <(const VariableFormulacaoPadrao& var) const {

    // type
    if((int)this->type() < (int) var.type())
      return true;
    else if( (int)this->type() > (int) var.type() )
      return false;
	
    // machine
    if (machine_ < var.machine())
        return true;
    else if (machine_ > var.machine())
        return false;

    // task
    if (task_ < var.task()) 
        return true;
    else if (task_ > var.task())
        return false;

	return false;
}

bool VariableFormulacaoPadrao::operator ==(const VariableFormulacaoPadrao& var) const {
   return !(*this < var) && !(var < *this);
}

std::string VariableFormulacaoPadrao::ToString() const
{
    std::stringstream ss;
    ss << "X";

	// machine
    if(machine_ >= 0) {
		ss << "_M" << machine_;
	}

	// task
	if(task_ >= 0) {
        ss << "_T" << task_;
    }

	return ss.str();
}

bool VariableFormulacaoPadraoHasher::operator()(const VariableFormulacaoPadrao& v1, const VariableFormulacaoPadrao& v2) const
{
   return (v1 < v2);
}

size_t VariableFormulacaoPadraoHasher::operator()(const VariableFormulacaoPadrao& var) const
{
    const int kHashPrime = 2654435761;
    unsigned int sum = 0;

	sum = (int)var.type();

    stdext::hash_compare<int> h;
	// machine
	if (var.machine() >= 0)
    {
      sum *= kHashPrime;
      sum += h.operator ()(var.machine());
    }
    
    
    // task
	if (var.task() >= 0)
    {
      sum *= kHashPrime;
      sum += h.operator ()(var.task());
    }

    return sum;
}


bool VariableFormulacaoPadraoPtrHasher::operator()(const VariableFormulacaoPadrao* v1, const VariableFormulacaoPadrao* v2) const
{
   return (*v1 < *v2);
}

size_t VariableFormulacaoPadraoPtrHasher::operator()(const VariableFormulacaoPadrao* var) const
{
	VariableFormulacaoPadraoHasher hasher;
	return hasher.operator()(*var);
}
