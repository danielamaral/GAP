#include "ConstraintGeracaoColunas.h"

#include <sstream>

ConstraintGeracaoColunas::ConstraintGeracaoColunas() {
	dualvalue_ = 1.0;
	Clear();
}

ConstraintGeracaoColunas::ConstraintGeracaoColunas(
    const ConstraintGeracaoColunas &cons) {
	dualvalue_ = 1.0;
	*this = cons;
}

ConstraintGeracaoColunas::~ConstraintGeracaoColunas() {
    Clear();
}

ConstraintGeracaoColunas& ConstraintGeracaoColunas::operator=(
    const ConstraintGeracaoColunas& cons) {   
	this->type_ = cons.type();
  this->machine_ = cons.machine();
  this->task_ = cons.task();
  this->dualvalue_ = cons.dualvalue();

  return *this;
}

bool ConstraintGeracaoColunas::operator<(
    const ConstraintGeracaoColunas& cons) const {
  if((int)type_ < (int) cons.type()) {
     return true;
  } else if((int)type_ > (int) cons.type()) {
     return false;
  }

  if (machine_ < cons.machine()) {
      return true;
  } else if (machine_ > cons.machine()) {
      return false;
  }

  if (task_ < cons.task()) {
      return true;
  } else if (task_ > cons.task()) {
      return false;
  }
	return false;
}

bool ConstraintGeracaoColunas::operator==(
    const ConstraintGeracaoColunas& cons) const {
	return (!(*this < cons) && !(cons < *this));
}

void ConstraintGeracaoColunas::Clear() {
	type_ = C_ERROR;
	machine_ = -1;
	task_ = -1;
	dualvalue_ = 1.0;
}

std::string ConstraintGeracaoColunas::ToString() const {
  std::stringstream ss;

	switch(type_)	{
    case C_SUM_ALLOCATIONS_PER_MACHINE_AT_MOST_ONE:
      ss << "C_SUM_ALLOCATIONS_PER_MACHINE_AT_MOST_ONE";
      break;
    case C_SUM_ALLOCATIONS_PER_TASK_EQUALS_ONE:
      ss << "C_SUM_ALLOCATIONS_PER_TASK_EQUALS_ONE";
      break;
    default:
      ss << "CError";			
      break;
	}

	// machine
	if(machine_ >= 0) {
    ss << "_M" << machine_;
  }

  // task
  if (task_ >= 0) {
    ss << "_T" << task_;
  }

	return ss.str();
}

size_t ConstraintGeracaoColunasHasher::operator()(
    const ConstraintGeracaoColunas& cons) const {
  const int kHashPrime = 2654435761;
  unsigned int sum = 0;
	sum = (int)cons.type();
  stdext::hash_compare<int> h;
	// machine
	if (cons.machine() >= 0) {
    sum *= kHashPrime;
    sum += h.operator ()(cons.machine());
  }

  // task
	if (cons.task() >= 0) {
    sum *= kHashPrime;
    sum += h.operator ()(cons.task());
  }
  return sum;
}

bool ConstraintGeracaoColunasHasher::operator()(
    const ConstraintGeracaoColunas& cons1,
    const ConstraintGeracaoColunas& cons2) const {
	return (cons1 < cons2);
}