#include "VariableGeracaoColunas.h"

#include <sstream>
#include <functional>

VariableGeracaoColunas::VariableGeracaoColunas() {
	Clear();
}

VariableGeracaoColunas::VariableGeracaoColunas(
    const VariableGeracaoColunas& var) {
    *this = var;
}

void VariableGeracaoColunas::Clear() {
  type_ = V_ERROR;
	machine_ = -1;
	task_ = -1;
  column_.Clear();
	value_ = 0.0;
	reducedcost_ = 0.0;
}

VariableGeracaoColunas::~VariableGeracaoColunas() {
	Clear();
}

VariableGeracaoColunas& VariableGeracaoColunas::operator=(const VariableGeracaoColunas& var) {
	type_ = var.type();
  machine_ = var.machine();
  task_ = var.task();
  column_ = var.column();
  value_ = var.value();
  reducedcost_ = var.reducedcost();
	return *this;
}

bool VariableGeracaoColunas::operator <(const VariableGeracaoColunas& var) const {
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

  // if COL type, tests the columns
  if (this->type() == COL) {
    if (this->column().machine() < var.column().machine())
      return true;
    else if (this->column().machine() > var.column().machine())
      return false;

    if (this->column().tasks() < var.column().tasks())
      return true;
    else if (this->column().tasks() > var.column().tasks())
      return false;

    if (this->column().index() < var.column().index())
      return true;
    else if (this->column().index() > var.column().index())
      return false;
  }

	return false;
}

bool VariableGeracaoColunas::operator ==(const VariableGeracaoColunas& var) const {
   return !(*this < var) && !(var < *this);
}

string VariableGeracaoColunas::ToString() const {
  stringstream ss;
  switch (type_) {
    case Z_k:
      ss << "Z";
      break;
    case W_k:
      ss << "W";
      break;
    case COL:
      ss << "COL";
      break;
    default:
      // TODO(danielrocha): log(error) here
      break;
  }

  if(machine_ >= 0) {
		ss << "_M" << machine_;
	}

	if(task_ >= 0) {
    ss << "_T" << task_;
  }

  if (type_ == COL) {
    ss << "_IDX" << column().index();
  }

	return ss.str();
}

bool VariableGeracaoColunasHasher::operator()(
    const VariableGeracaoColunas& v1, const VariableGeracaoColunas& v2) const {
   return (v1 < v2);
}

size_t VariableGeracaoColunasHasher::operator()(
    const VariableGeracaoColunas& var) const {
  const int kHashPrime = 2654435761;
  unsigned int sum = 0;

	sum = (int)var.type();
  stdext::hash_compare<int> h;
	// machine
	if (var.machine() >= 0) {
    sum *= kHashPrime;
    sum += h.operator ()(var.machine());
  }
	if (var.task() >= 0) {
    sum *= kHashPrime;
    sum += h.operator ()(var.task());
  }
  if (var.type() == VariableGeracaoColunas::COL && var.column().index() >= 0) {
    sum *= kHashPrime;
    sum += h.operator ()(var.column().index());
  }

  return sum;
}

bool VariableGeracaoColunasPtrHasher::operator()(
    const VariableGeracaoColunas* v1, const VariableGeracaoColunas* v2) const {
   return (*v1 < *v2);
}

size_t VariableGeracaoColunasPtrHasher::operator()(
    const VariableGeracaoColunas* var) const {
	VariableGeracaoColunasHasher hasher;
	return hasher.operator()(*var);
}