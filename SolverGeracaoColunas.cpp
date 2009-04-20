#include "SolverGeracaoColunas.h"

#include <cassert>

#include <algorithm>
#include <iostream>
#include <numeric>
#include <set>

#include "logging.h"
#include "minknap.c"
#include "opt_cplex.h"
#include "ProblemData.h"
#include "SolverFormulacaoPadrao.h"

SolverGeracaoColunas::SolverGeracaoColunas(ProblemData* problem_data) :
    VnsSolver(problem_data) {
  VLOG(2) << "Constructing new SolverGeracaoColunas";
  stopwatch_ = gcnew Stopwatch;
}

SolverGeracaoColunas::~SolverGeracaoColunas() {
  if (lp_ != NULL)
    delete lp_;
  VLOG(2) << "Deleting SolverGeracaoColunas";
}


/********************************************************************
**                       UTILITY METHODS                           **
*********************************************************************/
void SolverGeracaoColunas::Init(const SolverOptions& options) {
  // Reseta todas as variaveis internas.
  column_count_ = 0;
  total_num_nodes_ = 0;
  root_lower_bound_ = 0.0;
  node_with_best_result_ = -1;
  has_generated_initial_columns_ = false;
  cHash_.clear();
  stopwatch_->Reset();
  vContainer_.clear();
  column_map_.clear();

  stabilize_column_generation_ = options.use_stabilization();
  

  // O resolvedor a ser utilizado: é importante notar que o resolvedor é LP, ou
  // seja, vamos fazer B&B para buscar a solução ótima.
  lp_ = new OPT_CPLEX;
  lp_->createLP("FormulacaoGeracaoColunas", OPTSENSE_MINIMIZE, PROB_LP);
  //lp->setMIPScreenLog();
  //lp->setScreenLog(1);
  //lp->setSimplexScreenLog(1);
  lp_->setMIPEmphasis(0);
  lp_->setMIPRelTol(0.00);
	lp_->setMIPAbsTol(0.00);

  // Adiciona as restricoes ao LP.
  AddSumAssignmentsPerTaskEqualsOneConstraints(*problem_data_, &cHash_, lp_);
  AddSumAssignmentsPerMachineAtMostOneConstraints(*problem_data_, &cHash_, lp_);

  // Acha os valores das variáveis duais na formulação relaxada
  vector<double> dual_values;
  SolverFormulacaoPadrao::GetRelaxedDualValues(*problem_data_, &dual_values);
  VLOG(1) << "Got relaxed dual values. Size: " << dual_values.size();

  // Adiciona as colunas para a estabilizacao da geracao de colunas
  if (stabilize_column_generation_)
    AddStabilizationColumnsWithCoeficients(*problem_data_, dual_values, &vContainer_, lp_);
}

bool SolverGeracaoColunas::TimeExpired() const {
  return (time_limit_in_milliseconds_ > 0 &&
          stopwatch_->ElapsedMilliseconds > time_limit_in_milliseconds_);
}

// Calcula a matriz Xij com os valores fracionarios das
// associacoes de tarefas à maquinas. Em outras palavras, transforma
// a formulação de geração em colunas na formulação padrão calculando
// as variáveis binárias Xij.
void SolverGeracaoColunas::GetXijMatrix(
    const vector<double>& current_sol,
    const VariableGeracaoColunasContainer& vContainer,
    vector<vector<double> >* x) const {
  for (int v = 0; v < static_cast<int>(vContainer.size()); ++v) {
    const VariableGeracaoColunas& var = vContainer[v];
    int index = v;
    // Se a coluna i está sendo usada (mesmo que fracionalmente)
    if (var.type() == VariableGeracaoColunas::COL &&
        current_sol[index] > Globals::EPS()) {
      int machine = var.column().machine();
      DCHECK_GT(lp_->getCoef(problem_data_->num_tasks() + machine, index),
                1.0 - Globals::EPS())
        << "Column: " << index << ", machine: " << machine << ", row: "
        << problem_data_->num_tasks() + machine;
      DCHECK_GE(machine, 0);

      // Adiciona a fração que está sendo usada dessa alocação à Xij
      const vector<short>& tasks = GetColumnTasks(var.column().index());
      for (int task_idx = 0; task_idx < static_cast<int>(tasks.size()); ++task_idx) {
        (*x)[machine][tasks[task_idx]] += current_sol[index];
        DCHECK_GT(lp_->getCoef(tasks[task_idx], index), 1 - Globals::EPS())
          << "Column: " << index << ", machine: " << machine << ", task: "
          << tasks[task_idx] << ", num_tasks: " << tasks.size() << ", idx: "
          << task_idx;
      }
    } else if (var.type() == VariableGeracaoColunas::W_k ||
               var.type() == VariableGeracaoColunas::Z_k) {
      CHECK_GT(0.0 + Globals::EPS(), current_sol[index])
        << "GetXijMatrix: stabilization column in use: " << index
        << ", " << var.ToString();
    }
  }
}

void SolverGeracaoColunas::AddTasksToColumnMap(int column_index,
                                               const vector<short>& tasks) {
  column_map_[column_index] = tasks;
}

const vector<short>& SolverGeracaoColunas::GetColumnTasks(int column_index) const {
  map<int, vector<short> >::const_iterator it = column_map_.find(column_index);
  CHECK(it != column_map_.end())
    << "GetColumnTasks: column not found " << column_index;
  return it->second;
}

void SolverGeracaoColunas::RemoveColumnTasksFromMap(int column_index) {
  map<int, vector<short> >::iterator it = column_map_.find(column_index);
  CHECK(it != column_map_.end())
    << "RemoveColumnTasksFromMap: column not found " << column_index;
  column_map_.erase(it);
}

/********************************************************************
**                   COLUMN GENERATION METHODS                     **
*********************************************************************/
void SolverGeracaoColunas::RemoveExcessColumns(
    const ProblemData& pd,
    int num_columns_to_remove,
    VariableGeracaoColunasContainer* vContainer,
    OPT_LP* lp) {
  vector<double> reduced_costs(lp->getNumCols(), 0.0);
  lp->getDj(&reduced_costs[0]);
  VLOG(2) << "RemoveExcessColumns: got reduced costs for " << reduced_costs.size()
          << " columns.";

  vector<pair<double, int> > cost_and_index;
  for (int v = 0; v < static_cast<int>(vContainer->size()); ++v) {
    const VariableGeracaoColunas& var = (*vContainer)[v];
    // Só queremos remover as colunas geradas
    if (var.type() == VariableGeracaoColunas::COL) {
      int index = v;
      double cost = reduced_costs[index];
      cost_and_index.push_back(make_pair(cost, index));
    }
  }

  // Ordena de forma que o maior custo fique no começo.
  sort(cost_and_index.rbegin(), cost_and_index.rend());
  VLOG(2) << "RemoveExcessColumns: ordered columns by reduced costs.";

  int removed_columns = min<int>(num_columns_to_remove, cost_and_index.size());
  vector<int> removed_indices(removed_columns);
  vector<VariableGeracaoColunasContainer::iterator> removed_itr(removed_columns);
  for (int i = 0; i < removed_columns; ++i) {
    removed_indices[i] = cost_and_index[i].second;
    removed_itr[i] = (vContainer->begin() + removed_indices[i]);
  }

  VLOG(2) << "RemoveExcessColumns: removing excessive columns from LP and vContainer.";
  lp->delSetCols(removed_columns, &removed_indices[0]);
  for (int i = removed_columns - 1; i >= 0; --i) {
    RemoveColumnTasksFromMap(removed_itr[i]->column().index());
    vContainer->erase(removed_itr[i]);
  }
}

double SolverGeracaoColunas::GenerateColumns(const ProblemData& p,
                                             OPT_LP* lp,
                                             int* num_columns,
                                             vector<vector<short> >* fixed_vars,
                                             double best_solution_value) {

  VLOG(2) << "GenerateColumns: {num_columns: " << *num_columns
          << ", best_solution_value: " << best_solution_value << "}";
  // Em cada laço desse loop tentaremos gerar num_machines() colunas, uma para
  // cada máquina.
  double max_lower_bound = 0;
  double objective_value = 0.0;
  bool generated_and_added_column = true;
  while(generated_and_added_column) {
    generated_and_added_column = false;

    const int kNumLpMethods = 7;
    METHOD kMethodsToTry[kNumLpMethods] = {
      METHOD_AUTOMATIC, METHOD_SIFTING, METHOD_PRIMAL, METHOD_CONCURRENT, METHOD_DUAL,
      METHOD_BARRIERANDCROSSOVER, METHOD_BARRIERNOCROSSOVER };

    OPTSTAT status;
    for (int method = 0; method < kNumLpMethods; ++method) {
      VLOG(4) << "Trying method " << method << "...";
      status = lp->optimize(kMethodsToTry[method]);
      VLOG(4) << "Method " << kMethodsToTry[method] << ": status " << status
              << " obj " << lp->getObjVal();
      if (status == OPTSTAT_LPOPTIMAL || status == OPTSTAT_MIPOPTIMAL)
        break;
    }
    
    if (status != OPTSTAT_LPOPTIMAL && status != OPTSTAT_MIPOPTIMAL) {
      LOG(ERROR) << "No method could solve the problem, trying harder...";

      vector<double> x(lp->getNumCols());
      lp->getX(&x[0]);
      LOG(ERROR) << "GenerateColumns: invalid optimize status: " << lp->getStat()
                 << ", non-zero variables:\n";
      if (lp->getNumCols() != vContainer_.size()) {
        LOG(ERROR) << "getNumCols() : " << lp->getNumCols() << " != vContainer: "
                   << vContainer_.size();
      }
      for (int i = 0; i < lp->getNumCols(); ++i)
        if (x[i] > Globals::BigEPS())
          LOG(ERROR) << vContainer_[i].ToString() << ": " << x[i];
      lp->writeProbLP("SolverGeracaoColunas-infeasible");
      lp->writeProbMPS("SolverGeracaoColunas-infeasible");
      lp->setSimplexScreenLog(2);

      lp->readCopyMPS("SolverGeracaoColunas-infeasible.mps");
      LOG(ERROR) << "Trying again with the same OPT_LP, reloaded...";
      for (int method = 0; method < kNumLpMethods; ++method) {
        LOG(ERROR) << "Trying method " << method << "...";
        status = lp->optimize(kMethodsToTry[method]);
        LOG(ERROR) << "Method " << kMethodsToTry[method] << ": status " << status
                << " obj " << lp->getObjVal();
        if (status == OPTSTAT_LPOPTIMAL || status == OPTSTAT_MIPOPTIMAL)
          break;
      }

      // Loads up a new cplex
      OPT_LP* new_lp = new OPT_CPLEX;
      new_lp->setSimplexScreenLog(1);
      new_lp->readCopyMPS("SolverGeracaoColunas-infeasible.mps");
      LOG(ERROR) << "Trying again with a new OPT_LP, reloaded...";
      for (int method = 0; method < kNumLpMethods; ++method) {
        LOG(ERROR) << "Trying method " << method << "...";
        status = new_lp->optimize(kMethodsToTry[method]);
        LOG(ERROR) << "Method " << kMethodsToTry[method] << ": status " << status
                << " obj " << new_lp->getObjVal();
        if (status == OPTSTAT_LPOPTIMAL || status == OPTSTAT_MIPOPTIMAL)
          break;
      }
      delete new_lp;

      return Globals::Infinity();
      //assert(false);
    }

    objective_value = lp->getObjVal();
    VLOG_EVERY_N(3, 50)
      << "GenerateColumns: loop start, objective_value: " << objective_value
      << ", status: " << status;

    if (TimeExpired()) {
      VLOG(2) << "GenerateColumns: time expired, returning " << objective_value;
      return objective_value;
    }
    //lp->writeProbLP("SolverGeracaoColunas-gencol");

    // Mantem um limite no numero de colunas utilizadas: se for maior
    // que 40000, deleta 10000 colunas (escolhendo as colunas que tem o maior
    // custo reduzido) e re-otimiza.
    if(lp->getNumCols() > kMaxNumberColumns_) {
      VLOG(1) << "GenerateColumns: excessive number of columns: "
              << lp->getNumCols();
      RemoveExcessColumns(p, kNumColumnsToRemove_, &vContainer_, lp);
      lp->optimize(METHOD_PRIMAL);
    }
    
    // Os valores duais servirão para setar os preços para o subproblema
    vector<double> dual_values(lp->getNumRows(), 0.0);
    lp->getPi(&dual_values[0]);
    if (VLOG_IS_ON(5)) {
      stringstream dual_string;
      for (int i = 0; i < static_cast<int>(dual_values.size()); ++i)
        dual_string << dual_values[i] << ",";
      VLOG(5) << "GenerateColumns: dual values: " << dual_string.str();
    }
    
    // Custos reduzidos calculados a partir da solução da mochila
    vector<double> reduced_costs(p.num_machines());
    vector<int> prices(p.num_tasks());

    // Tenta gerar uma coluna para cada máquina
    for (int machine = 0; machine < p.num_machines(); ++machine) {
      int used_capacity = 0;

      // Os precos de cada tarefa para o problema da mochila, ajustados
      // em relação à fixação ou não das variáveis.
      for (int task = 0; task < p.num_tasks(); ++task) {
        prices[task] = (static_cast<int>(dual_values[task] * 100.0) -
                        (p.cost(machine, task) * 100));

        // Se o preco for negativo ou a alocacao maquina/tarefa ja
        // estiver fixada em zero, o preco é zero.
        if(prices[task] < 0 || (*fixed_vars)[machine][task] == 0)
          prices[task] = 0;

        // Se a alocacao maquina/tarefa ja estiver fixada em um,
        // o preco é zero e diminuimos a capacidade da maquina.
        if((*fixed_vars)[machine][task] == 1) {
          prices[task] = 0;
          used_capacity += p.consume(machine, task);
        }
      }

      vector<int> knapsack_result(p.num_tasks());
      int knapsack_value = minknap(p.num_tasks(), &prices[0],
                                   const_cast<int*>(p.GetConsumeVector(machine)),
                                   &knapsack_result[0],
                                   p.capacity(machine) - used_capacity);

      // Adiciona à solução encontrada as variáveis fixadas
      for(int task = 0; task < p.num_tasks(); ++task) {
        if (prices[task] == 0 || (*fixed_vars)[machine][task] == 0)
          knapsack_result[task] = 0;
        if ((*fixed_vars)[machine][task] == 1)
          knapsack_result[task] = 1;
      }

      double knapsack_price = 0.0;
      for(int task = 0; task < p.num_tasks(); ++task) {
        knapsack_price += dual_values[task] * knapsack_result[task];
      }
      knapsack_price += dual_values[p.num_tasks() + machine];

      reduced_costs[machine] =
        p.AssignmentCost(machine, knapsack_result) - knapsack_price;

      // Se a coluna gerada tem custo reduzido negativo, adiciona à solução
      if(reduced_costs[machine] < -Globals::EPS()) {
        CHECK_LE(p.AssignmentConsume(machine, knapsack_result), p.capacity(machine))
          << "The knapsack solution consumes more than the machine capacity!";

        generated_and_added_column = true;
        // Adiciona a nova coluna, com valor na função objetivo igual ao custo
        // da resposta da mochila.
        vector<short> knapsack_tasks;
        for (short task = 0; task < p.num_tasks(); ++task)
          if (knapsack_result[task] > 0)
            knapsack_tasks.push_back(task);
        AddNewColumnWithTasks(p.AssignmentCost(machine, knapsack_result), 0.0, OPT_INF,
                              machine, knapsack_tasks, &vContainer_, lp);
        *num_columns += 1;

        VLOG_EVERY_N(3, 197)
          << "GenerateColumns: new column: {machine: " << machine << ", reduced_cost: "
          << reduced_costs[machine] << ", price: " << knapsack_price
          << ", num_columns: " << *num_columns << ", objective_value: "
          << objective_value << "}";
      }
    }
    
    if(generated_and_added_column) {
      double sum_reduced_costs = accumulate(reduced_costs.begin(),
                                            reduced_costs.end(), 0.0);

      // Se gerou coluna para todos as máquinas e encontrou um lower bound
      // melhor (maior), guarda o lower bound
      if(max_lower_bound < objective_value + sum_reduced_costs)
        max_lower_bound = objective_value + sum_reduced_costs;

      // Se o lower bound encontrado for maior que o upper bound, pode retornar
      if(objective_value + sum_reduced_costs >=
         best_solution_value - 1 + Globals::BigEPS()) {
        VLOG(3) << "GenerateColumns: end, obj_val + reduced_costs >= best solution: "
                << objective_value << " + " << sum_reduced_costs << " >= "
                << best_solution_value +1 - Globals::BigEPS();
        return objective_value + sum_reduced_costs;
      }

      VLOG_EVERY_N(3, 50)
        << "GenerateColumns: added at least one column: {sum_reduced_costs: "
        << sum_reduced_costs << ", max_lower_bound: " << max_lower_bound << "}";
    }
  }

  /*if (VLOG_IS_ON(2)) {
    VLOG(2) << "Optimize 1 status: " << lp->optimize(METHOD_PRIMAL);
    vector<double> x(lp->getNumCols()); lp->getX(&x[0]);
    stringstream s;
    for (int i = 0; i < lp->getNumCols(); ++i)
      s << ", " << vContainer_[i].ToString() << ": " << x[i];
    VLOG(2) << s.str();
  }*/

  VLOG(3) << "GenerateColumns: end, returning " << objective_value;
  //lp->writeProbLP("SolverGeracaoColunas-gercol-end");
  return objective_value;
}

void SolverGeracaoColunas::AdjustStabilizationBounds(
    const ProblemData& p, double bound,
    const VariableGeracaoColunasContainer& vContainer, OPT_LP* lp) {
  VLOG(1) << "Adjusting stabilization bounds to " << bound;
  vector<double> variable_bounds(2*(p.num_tasks() + p.num_machines()), bound);
  vector<BOUNDTYPE> bound_types(2*(p.num_tasks() + p.num_machines()), BOUND_UPPER);
  vector<int> variable_indices(2*(p.num_tasks() + p.num_machines()));
  int c = 0;
  for (int v = 0; v < static_cast<int>(vContainer.size()); ++v) {
    const VariableGeracaoColunas& var = vContainer[v];
    int index = v;
    if (var.type() == VariableGeracaoColunas::Z_k ||
        var.type() == VariableGeracaoColunas::W_k) {
      variable_indices[c++] = index;
    }
  }
  CHECK_EQ(c, 2 * (p.num_tasks() + p.num_machines()))
    << "Wrong number of stabilization variables in vContainer";
  lp->chgBds(c, &variable_indices[0], &bound_types[0], &variable_bounds[0]);

  /*if (VLOG_IS_ON(2)) {
    VLOG(2) << "Optimize 4 status: " << lp->optimize(METHOD_PRIMAL);
    vector<double> x(lp->getNumCols()); lp->getX(&x[0]);
    stringstream s;
    for (int i = 0; i < lp->getNumCols(); ++i)
      s << ", " << vContainer_[i].ToString() << ": " << x[i];
    VLOG(2) << s.str();
  }*/
}

void SolverGeracaoColunas::UpdateStabilizationCosts(
    const ProblemData& p, const vector<double>& dual_values,
    VariableGeracaoColunasContainer* vContainer, OPT_LP* lp) {
  VLOG(1) << "Updating stabilization costs";
  for (int v = 0, c = 0; v < static_cast<int>(vContainer->size()); ++v) {
    VariableGeracaoColunas* var = &(*vContainer)[v];
    int index = v;
    if (var->type() == VariableGeracaoColunas::Z_k) {
      lp->chgObj(index, +1.0 * dual_values[c]);
      var->set_cost(+1.0 * dual_values[c]);
      ++c;
    }
  }
  for (int v = 0, c = 0; v < static_cast<int>(vContainer->size()); ++v) {
    VariableGeracaoColunas* var = &(*vContainer)[v];
    int index = v;
    if (var->type() == VariableGeracaoColunas::W_k) {
      lp->chgObj(index, -1.0 * dual_values[c]);
      var->set_cost(-1.0 * dual_values[c]);
      ++c;
    }
  }
  /*if (VLOG_IS_ON(2)) {
    VLOG(2) << "Optimize 3 status: " << lp->optimize(METHOD_PRIMAL);
    vector<double> x(lp->getNumCols()); lp->getX(&x[0]);
    stringstream s;
    for (int i = 0; i < lp->getNumCols(); ++i)
      s << ", " << vContainer_[i].ToString() << ": " << x[i];
    VLOG(2) << s.str();
  }*/
}

double SolverGeracaoColunas::GenerateColumnsWithStabilization(
    const ProblemData& p, OPT_LP* lp, int* num_columns,
    vector<vector<short> >* fixed_vars, double best_solution_value) {
  int kNumStabilizationBounds = 4;
  static double kStabilizationBounds[] = { 0.1, 0.01, 0.001, 0.0 };
  double lower_bound = 0.0;
  for (int b = 0; b < kNumStabilizationBounds; ++b) {
    if (b == 0) {
      AdjustStabilizationBounds(p, kStabilizationBounds[b], vContainer_, lp);
    } else {
      vector<double> dual_values(lp->getNumRows(), 0.0);
      lp->getPi(&dual_values[0]);
      AdjustStabilizationBounds(p, kStabilizationBounds[b], vContainer_, lp);
      UpdateStabilizationCosts(p, dual_values, &vContainer_, lp);
    }

    double new_lower_bound =
      GenerateColumns(p, lp, num_columns, fixed_vars, best_solution_value);
    if (b > 0)
      CHECK_GE(new_lower_bound, lower_bound - Globals::BigEPS())
        << "Stabilization: new_lower_bound (" << new_lower_bound << ") < "
        << "lower_bound (" << lower_bound << ")";

    // O 'max' abaixo só é necessário devido a problemas de precisão numérica
    lower_bound = max<double>(lower_bound, new_lower_bound);
    if (lower_bound > best_solution_value - 1.0 + Globals::BigEPS()) {
      VLOG(1) << "Stabilization: lower_bound > best_solution_value: "
              << lower_bound << " > " << best_solution_value - 1.0 + Globals::BigEPS();
      return lower_bound;
    }

    /*if (VLOG_IS_ON(2)) {
      VLOG(2) << "Optimize 2 status: " << lp->optimize(METHOD_PRIMAL);
      vector<double> x(lp->getNumCols()); lp->getX(&x[0]);
      stringstream s;
      for (int i = 0; i < lp->getNumCols(); ++i)
        s << ", " << vContainer_[i].ToString() << ": " << x[i];
      VLOG(2) << s.str();
    }*/

    if (TimeExpired()) {
      VLOG(2) << "Stabilization: time expired, returning " << lower_bound;
      return lower_bound;
    }
  }
  VLOG(2) << "Stabilization: returning " << lower_bound;
  return lower_bound;
}


/********************************************************************
**                    BRANCH AND BOUND METHODS                     **
*********************************************************************/

OPTSTAT SolverGeracaoColunas::SetUpAndRunBranchAndBound(
    const ProblemData& p,
    int num_nodes_limit,
    OPT_LP* lp,
    ProblemSolution* integer_solution) {
  int num_columns = 0;
  double pct_tree_solved = 0.0;
  int num_visited_nodes = 0;

  // Cria uma matriz de variáveis fixadas, deixando inicialmente todas não fixadas
  vector<vector<short> > fixed_vars(p.num_machines(),
                                    vector<short>(p.num_tasks(), -1));

  OPTSTAT status = OPTSTAT_NOINTEGER;
  BB(p,
     num_nodes_limit,  // limiteNos = 1000000
     0,  // profundidade = 0
     0,  // Lower bound = 0
     lp,
     integer_solution,  // melhorSolucao = integer_solution
     &fixed_vars,  // fixado = fixed
     &num_columns,  // contCol = 0
     &pct_tree_solved,  // acumulador = 0.0
     &num_visited_nodes,  // nosVisitados = 0
     //cortes,  
     &status);  // status = OPTSTAT_NOINTEGER

  return status;
}

double SolverGeracaoColunas::BB(const ProblemData& p,
                                int num_nodes_limit,
                                int depth,
                                double lower_bound,
                                OPT_LP* lp,
                                ProblemSolution* best_solution,
                                vector<vector<short> >* fixed_vars,
                                int* num_columns,
                                double* pct_tree_solved,
                                int* num_visited_nodes,
                                OPTSTAT* status) {
  //tLpNo.reset();
  //tMochilaNo.reset();

  VLOG(2) << "BB(" << depth << "): {num_visited_nodes: " << *num_visited_nodes
          << ", pct_tree_solved: " << *pct_tree_solved << ", lower_bound: "
          << lower_bound << "}";
  double new_lower_bound;
  if (stabilize_column_generation_) {
    new_lower_bound = GenerateColumnsWithStabilization(
        p, lp, num_columns, fixed_vars, best_solution->cost());
  } else {
    new_lower_bound = GenerateColumns(
        p, lp, num_columns, fixed_vars, best_solution->cost());
  }
  VLOG(2) << "BB(" << depth << "): column generation lower bound: "
          << new_lower_bound;
  if (depth == 0) {
    LOG(INFO) << "BB(" << depth << "): root node lower bound: " << new_lower_bound;
    root_lower_bound_ = new_lower_bound;
  } else {
    ++total_num_nodes_;
  }

  *num_visited_nodes += 1;
  if(*num_visited_nodes >= num_nodes_limit) {
    VLOG(1) << "BB(" << depth << "): num_visited_nodex > limit: "
            << *num_visited_nodes << ", return: " << new_lower_bound;
    return new_lower_bound;
  }

  // Se o melhor LB encontrado for maior que a melhor solução encontrada
  if(new_lower_bound >= best_solution->cost() - 1.0 + Globals::BigEPS()) {
    *pct_tree_solved += pow(0.5, static_cast<double>(depth));
    if(*pct_tree_solved >= 1.0 - Globals::EPS()) {
      // Se a porcentagem do BB já resolvido for >= 1 ajusta resposta
      *status = OPTSTAT_MIPOPTIMAL;
    }
    VLOG(1) << "BB(" << depth << "): lower_bound > best_solution, pct_tree_solved: "
            << *pct_tree_solved << ", status = " << *status;
    return new_lower_bound;
  }

  if (TimeExpired()) {
    VLOG(2) << "BB(" << depth << "): time expired (after GenerateColumns), returning "
            << new_lower_bound;
    return new_lower_bound;
  }

  CHECK_GE(new_lower_bound + Globals::EPS(), lower_bound)
    << "The column generation lower bound must be at least as good as the input "
    << "bound.";

  vector<double> current_solution(lp->getNumCols());
  lp->getX(&current_solution[0]);

  /*
  //verificacoes redundantes
  bool temPau = false;  
  for(i = 0; i < p.numTrabalhos; i++) {
    if(solCorrente[i] > eps)  {
      temPau = true;
      printf("Coluna IMPROPRIA: %d  Valor: %lf\n",i,solCorrente[i]);
    }
  }

  if(temPau) {
    printf("Infeasible\n");
    acumulador += pow(0.5,(double)profundidade);
    #ifdef MENSAGEM_BB 
      printf("Total da Arvore Resolvida: %lf\n--------------------------------\n",acumulador);
    #endif
    if(acumulador > 1 - eps)
    {
      if(status == BB_NO_INTEGER)
        status = BB_INFEASIBLE;
      else if(status == BB_INTEGER)
        status = BB_OPTIMAL;
    }
    return LB;    
  }

  
  for(i = 2 + p.numTrabalhos; i < 2 + p.numTrabalhos + (p.numTrabalhos+p.numAgentes)*2; i++)
  {
    if(solCorrente[i] > eps)
    {      
      printf("Coluna ESTABILIZACAO USADA: %d  Valor: %lf\n",i,solCorrente[i]);
      exit(0);
    }
  }
  //fim verificação redundantes
  */

  vector<vector<double> > x(p.num_machines(),
                            vector<double>(p.num_tasks(), 0.0));
  GetXijMatrix(current_solution, vContainer_, &x);
  VLOG(1) << "BB(" << depth << "): got current solution and generated Xij matrix.";

  // Calcula o custo da solução fracionária representada por Xij
  /*double relaxed_solution_cost = 0.0;
  for (int machine = 0; machine < p.num_machines(); ++machine)
    for (int task = 0; task < p.num_tasks(); ++task)
      relaxed_solution_cost += p.cost(machine, task) * x[machine][task];
  */
  
  //determinando quem serah fixado
  int num_integer_vars = 0;
  int num_variables_on_one = 0;
  double current_cost = 0.0;

  // TODO(danielrocha): move parameters to a better place
  static const int kMaxDepthFixingVars_ = 4;
  static const int kNumberOfLookupsFixingVars_[] = { 10, 8, 6, 4 };
  //static const int kNumberOfLookupsFixingVars_[] = { 15, 5, 3, 2 };
  //static const int kNumberOfLookupsFixingVars_[] = { 4, 3, 2, 1 };

  int fixed_machine;
  int fixed_task;
  if(depth < kMaxDepthFixingVars_) {
    VLOG(1) << "BB(" << depth << "): calling SelectFixedVariables to select variables.";
    num_integer_vars =
      SelectFixedVariable(p, kNumberOfLookupsFixingVars_[depth],
                          x, lp, best_solution, fixed_vars, num_columns,
                          &fixed_machine, &fixed_task);
  } else {
    VLOG(1) << "BB(" << depth << "): selecting variables using heuristic.";
    double bestVal = 0.0;
    for (int machine = 0; machine < p.num_machines(); ++machine)
      for (int task = 0; task < p.num_tasks(); ++task)
        if(x[machine][task] > 1.0 - Globals::EPS() ||
           x[machine][task] < Globals::EPS()) {
          ++num_integer_vars;
          if (x[machine][task] > 1.0 - Globals::EPS()) {
            ++num_variables_on_one;
            current_cost += p.cost(machine, task);
          }
        } else {
          double heuristic_eval = EvaluateVariableToFix(x[machine][task]);
          if (bestVal < heuristic_eval) {
            bestVal = heuristic_eval;
            fixed_machine = machine;
            fixed_task = task;
          }
        }
  }
  VLOG(1) << "BB(" << depth << "): fixed task: " << fixed_task << " and machine: "
          << fixed_machine;

  // Se o numero de variaveis inteiras é igual ao total de variáveis, acabou-se
  // o B&B.
  if (num_integer_vars == p.num_machines() * p.num_tasks()) {
    *status = OPTSTAT_FEASIBLE;
    if (lp->getObjVal() < best_solution->cost()) {
      best_solution->Clear();
      for(int machine = 0; machine < p.num_machines(); ++machine) {
        for(int task = 0; task < p.num_tasks(); ++task) {
          if (x[machine][task] > 1.0 - Globals::EPS())
            best_solution->set_assignment(task, machine);
        }
      }
      CHECK(best_solution->IsValid());
      CHECK_EQ(best_solution->cost(), static_cast<int>(lp->getObjVal() + Globals::EPS()))
        << "Best BB solution must be equal to lp->getObjVal()";
      LOG(INFO) << "BB(" << depth << "): Found best integer solution: "
                << best_solution->cost() << ", lower bound: " << new_lower_bound;
      node_with_best_result_ = total_num_nodes_;
    }
    
    *pct_tree_solved += pow(0.5, static_cast<double>(depth));
    // Se temos todas as variáveis inteiras e já resolvemos toda a árvore, 
    // temos a solução ótima.
    if(*pct_tree_solved > 1.0 - Globals::EPS()) {
      *status = OPTSTAT_MIPOPTIMAL;
    }
    VLOG(1) << "BB(" << depth << "): all integer: {pct_tree_solved: "
            << *pct_tree_solved << ", status: " << *status << "}";
    return new_lower_bound;
  } else {
    // Calcula uma aproximacao do quanto resta adicionar ao custo da solucao
    if ((p.num_tasks() - num_variables_on_one) * p.MinimumAssignmentCost() + current_cost >
         best_solution->cost()) {
      LOG(INFO) << "BB(" << depth << "): estimation shows that this node "
                << "is not promising: num_variables_on_one: " << num_variables_on_one
                << ", current_cost: " << current_cost << ", minimum assignment cost: "
                << p.MinimumAssignmentCost() << ", best solution: "
                << best_solution->cost();
      return new_lower_bound;
    }
  }

  VLOG(1) << "BB(" << depth << "): number of integer variables: "
          << num_integer_vars;

  FixVariableAndContinueBB(FIX_ON_ZERO,
                           fixed_machine,
                           fixed_task,
                           num_nodes_limit,
                           new_lower_bound,
                           p,
                           lp,
                           best_solution,
                           fixed_vars,
                           num_columns,
                           depth,
                           pct_tree_solved,
                           num_visited_nodes,
                           status);

  if (TimeExpired()) {
    VLOG(2) << "BB(" << depth << "): time expired (after FixAndContinue), returning "
            << new_lower_bound;
    return new_lower_bound;
  }
  
  FixVariableAndContinueBB(FIX_ON_ONE,
                           fixed_machine,
                           fixed_task,
                           num_nodes_limit,
                           new_lower_bound,
                           p,
                           lp,
                           best_solution,
                           fixed_vars,
                           num_columns,
                           depth,
                           pct_tree_solved,
                           num_visited_nodes,
                           status);

  VLOG(1) << "BB(" << depth << "): end, return value: " << new_lower_bound;
  return new_lower_bound;
}

void SolverGeracaoColunas::SetAndStoreFixedVariables(
    FixingSense sense, const ProblemData& p, int fixed_machine, int fixed_task,
    vector<vector<short> >* fixed, vector<short>* before_fixing) {
  for (int machine = 0; machine < p.num_machines(); ++machine) {
    (*before_fixing)[machine] = (*fixed)[machine][fixed_task];
  }
  switch (sense) {
    case FIX_ON_ZERO:
      (*fixed)[fixed_machine][fixed_task] = 0;
      break;
    
    case FIX_ON_ONE:
      for (int machine = 0; machine < p.num_machines(); ++machine)
        (*fixed)[machine][fixed_task] = 0;
      (*fixed)[fixed_machine][fixed_task] = 1;
      break;
    
    default:
      LOG(FATAL) << "Invalid type of FixingSense: " << sense;
  }
}

bool SolverGeracaoColunas::ShouldRemoveColumnWhenFixing(
    FixingSense sense, const VariableGeracaoColunas& var, int column_number,
    int fixed_machine, int fixed_task) {
  switch (sense) {
    case FIX_ON_ZERO: {
      // Se a coluna 'column_number' se refere à máquina 'fixed_machine'
      if (fixed_machine == var.column().machine()) {
        CHECK_GT(lp_->getCoef(problem_data_->num_tasks() + fixed_machine, column_number),
                 1.0 - Globals::EPS())
          << "In the LP, column " << column_number
          << " is not associated with machine " << fixed_machine;
        const vector<short>& tasks = GetColumnTasks(var.column().index());
        // Se nessa coluna 'fixed_task' esta associada à 'fixed_machine'
        if (find(tasks.begin(), tasks.end(), fixed_task) != tasks.end()) {
          CHECK_GT(lp_->getCoef(fixed_task, column_number), 1.0 - Globals::EPS())
            << "In the LP, task " << fixed_task << " is not associated with "
            << "machine " << fixed_machine;
          //CHECK_DOUBLE_EQ(lp_->getObj(column_number), var.cost())
          //  << "In the LP, the cost of column " << column_number << " does not match "
          //  << "the internal representation";
          return true;
        }
      }
      return false;
    }

    case FIX_ON_ONE: {
      const vector<short>& tasks = GetColumnTasks(var.column().index());
      bool machine_on_column = fixed_machine == var.column().machine();
      bool task_on_column =
        find(tasks.begin(), tasks.end(), fixed_task) != tasks.end();
      if ((machine_on_column && !task_on_column) ||
          (!machine_on_column && task_on_column)) {
        int constraint_line = problem_data_->num_tasks() + fixed_machine;
        bool machine_on_column_lp =
          (lp_->getCoef(constraint_line, column_number) < Globals::EPS() &&
           lp_->getCoef(fixed_task, column_number) > 1 - Globals::EPS());
        bool task_on_column_lp = 
          (lp_->getCoef(constraint_line, column_number) > 1.0 - Globals::EPS() &&
           lp_->getCoef(fixed_task, column_number) < Globals::EPS());
        CHECK(machine_on_column_lp || task_on_column_lp)
          << "Machine or task is not on expected column: machine "
          << machine_on_column_lp << " task " << task_on_column_lp;
        return true;
      }
      return false;
    }

    default:
      LOG(FATAL) << "Invalid type of FixingSense: " << sense;
  }
  return false;
}

double SolverGeracaoColunas::FixVariableAndContinueBB(
    FixingSense fixing_sense, int fixed_machine, int fixed_task,
    int num_nodes_limit, double lower_bound,
    const ProblemData& p, OPT_LP* lp,
    ProblemSolution* best_solution,
    vector<vector<short> >* fixed_vars, int* num_columns, int depth,
    double* pct_tree_solved, int* num_visited_nodes, OPTSTAT* status) {
  // Salva todas as colunas que serão removidas com a fixação
  // Fixa a variável e chama BB()
  // Retorna todas as colunas previamente removidas

  vector<short> before_fixing(p.num_machines());
  SetAndStoreFixedVariables(fixing_sense, p, fixed_machine,
                            fixed_task, fixed_vars, &before_fixing);
  VLOG(2) << "FixVariableAndContinueBB: {fixed_machine: " << fixed_machine
          << ", fixed_task: " << fixed_task << ", value: " << fixing_sense
          << "}";

  // Calcula quantas colunas serão removidas
  vector<VariableGeracaoColunas> removed_vars;
  vector<VariableGeracaoColunasContainer::iterator> removed_itr;
  vector<int> removed_indices;
  for (int v = 0; v < static_cast<int>(vContainer_.size()); ++v) {
    const VariableGeracaoColunas& var = vContainer_[v];
    int index = v;
    if (var.type() == VariableGeracaoColunas::COL &&
        ShouldRemoveColumnWhenFixing(fixing_sense, var, index,
                                     fixed_machine, fixed_task)) {
      removed_indices.push_back(index);
      removed_vars.push_back(var);
      removed_itr.push_back(vContainer_.begin() + v);
    }
  }

  // Actually removes the variables
  lp->delSetCols(removed_indices.size(), &removed_indices[0]);
  for (int i = removed_itr.size() - 1; i >= 0; --i) {
    vContainer_.erase(removed_itr[i]);
  }

  // Chama o Branch and Bound
  double retorno = BB(p, num_nodes_limit, depth + 1, lower_bound, lp,
                      best_solution, fixed_vars,
                      num_columns, pct_tree_solved, num_visited_nodes,
                      status);

  // Retorna as variáveis ao estado original
  for (int machine = 0; machine < p.num_machines(); ++machine)
    (*fixed_vars)[machine][fixed_task] = before_fixing[machine];
  for (int i = 0; i < static_cast<int>(removed_vars.size()); ++i) {
    VariableGeracaoColunas* var = &removed_vars[i];
    AddNewColumnReuseTasks(*var, 0.0, OPT_INF, &vContainer_, lp);
  }

  return retorno;
}

double SolverGeracaoColunas::EvaluateVariableToFix(
    double binary_var) {
  if (binary_var > 1.0 - Globals::EPS() || binary_var < Globals::EPS())
    return 0.0;
  return (1 - fabs(binary_var - 0.5));
}

int SolverGeracaoColunas::SelectFixedVariable(
    const ProblemData& p, int num_vars_lookup,
    const vector<vector<double> >& x, OPT_LP* lp,
    ProblemSolution* best_solution, vector<vector<short> >* fixed_vars,
    int* num_columns, int* fixed_machine, int* fixed_task) {

  // Armazenando os num_vars_lookup melhores (maior value)
  set<FixingCandidate> candidates;
  int num_integer_vars = 0;
  for(int mac = 0; mac < p.num_machines(); ++mac) {
    for(int task = 0; task < p.num_tasks(); ++task) {
      if(x[mac][task] > 1.0 - Globals::EPS() || x[mac][task] < Globals::EPS()) {
        num_integer_vars++;
      } else {
        FixingCandidate new_candidate;
        new_candidate.task = task;
        new_candidate.machine = mac;
        new_candidate.value = EvaluateVariableToFix(x[mac][task]);
        if (new_candidate.value != 0.0) {
          if (static_cast<int>(candidates.size()) == num_vars_lookup &&
              new_candidate.value < candidates.begin()->value) {
            continue;
          }
          candidates.insert(new_candidate);
          if (static_cast<int>(candidates.size()) > num_vars_lookup)
            candidates.erase(candidates.begin());
        }
      }
    }
  }

  // Faz um pequeno B&B para cada candidato e guarda o melhor valor encontrado.
  double best_value = 0.0;
  for (set<FixingCandidate>::const_iterator it = candidates.begin();
       it != candidates.end(); ++it) {
    const FixingCandidate& candidate = *it;
 
    int num_visited_nodes = 0;
    double pct_tree_solved = 0.0;
    OPTSTAT status = OPTSTAT_NOINTEGER;

    double fixed_on_zero =
      FixVariableAndContinueBB(FIX_ON_ZERO, candidate.machine, candidate.task, 1,
                               1000000, p, lp, best_solution, fixed_vars,
                               num_columns, 0, &pct_tree_solved,
                               &num_visited_nodes, &status);

    num_visited_nodes = 0;
    pct_tree_solved = 0.0;
    double fixed_on_one =
      FixVariableAndContinueBB(FIX_ON_ONE, candidate.machine, candidate.task, 1,
                               1000000, p, lp, best_solution, fixed_vars,
                               num_columns, 0, &pct_tree_solved,
                               &num_visited_nodes, &status);
    // TODO(danielrocha): porque diabos otimizar aqui??
    lp->optimize(METHOD_AUTOMATIC);

    double heuristic_value =
      fixed_on_zero + fixed_on_one + min<double>(fixed_on_zero, fixed_on_one) * 7;

    /*printf("X[%3d][%3d]=%7.3lf %5d %7.3lf %7.3lf %7.3lf\n",
           candidate.machine, candidate.task, x[candidate.machine][candidate.task],
           p.custos[candidate.machine][candidate.task], fixed_on_zero, fixed_on_one,
           heuristic_value);*/

    if(heuristic_value > best_value) {
      *fixed_machine = candidate.machine;
      *fixed_task = candidate.task;
      best_value = heuristic_value;
    }
    if (TimeExpired()) {
      VLOG(2) << "SelectFixedVariable: time expired, returning: fixed_machine: "
              << *fixed_machine << ", fixed_task: " << *fixed_task << ", value: "
              << best_value << ", num_integer_vars: " << num_integer_vars;
      return num_integer_vars;
    }
  }

  return num_integer_vars;
}

/********************************************************************
**             VARIABLE CREATION + STABILIZATION                   **
*********************************************************************/

OPTSTAT SolverGeracaoColunas::AddInitialColumns(bool first, int time_limit,
                                                ProblemSolution* integer_solution) {
  LOG(INFO) << "Using " << time_limit << "s to find the first solution.";
  OPTSTAT first_status = SolverFormulacaoPadrao::GetIntegerSolutionWithTimeLimit(
      *problem_data_, time_limit, integer_solution);
  LOG(INFO) << "Got first integer solution, with cost " << integer_solution->cost();

  // Adiciona a solução inteira ao modelo
  AddIntegerSolutionColumnsWithCoeficients(*problem_data_,
                                           *integer_solution, &vContainer_, lp_);
  lp_->optimize(METHOD_AUTOMATIC);

  return first_status;
}

void SolverGeracaoColunas::AddStabilizationColumnsWithCoeficients(
    const ProblemData& p, const vector<double>& relaxed_dual_values,
    VariableGeracaoColunasContainer* vContainer, OPT_LP* lp) {
  const double epsD = 0.1;
  // Adiciona a identidade positiva para a estabilização da geração de colunas
  // às restrições do tipo SUM(vy) = 1 e SUM(y) <= 1.
  for (int index = 0; index < p.num_tasks() + p.num_machines(); ++index) {
    VariableGeracaoColunas var;
    var.set_type(VariableGeracaoColunas::Z_k);
    if (index < p.num_tasks())
      var.set_task(index);
    else
      var.set_machine(index - p.num_tasks());
    var.set_cost(relaxed_dual_values[index]);

    // Adds the column
    // TODO(danielrocha): adjust the bounds for stabilization
    vContainer->push_back(var);
    double lower_bound = 0.0, upper_bound = 0.1;
    lp->newCol(
      OPT_COL(OPT_COL::VAR_CONTINUOUS, var.cost(), lower_bound, upper_bound),
      var.ToString().c_str());
      //NULL);
    CHECK_EQ(vContainer->size(), lp->getNumCols());

    // Adjusts the matrix coeficient
    // TODO(danielrocha): maybe look for the constraint in the hash
    lp->chgCoef(index, lp->getNumCols() - 1, 1.0);
  }

  // Adiciona a identidade negativa para a estabilização da geração de colunas
  // às restrições do tipo SUM(vy) = 1.
  for (int index = 0; index < p.num_tasks() + p.num_machines(); ++index) {
    VariableGeracaoColunas var;
    var.set_type(VariableGeracaoColunas::W_k);
    if (index < p.num_tasks())
      var.set_task(index);
    else
      var.set_machine(index - p.num_tasks());
    var.set_cost(-1.0 * relaxed_dual_values[index]);

    // Adds the column
    // TODO(danielrocha): adjust the bounds for stabilization
    vContainer->push_back(var);
    double lower_bound = 0.0, upper_bound = 0.1;
    lp->newCol(
      OPT_COL(OPT_COL::VAR_CONTINUOUS, var.cost(), lower_bound, upper_bound),
      var.ToString().c_str());
      //NULL);
    CHECK_EQ(vContainer->size(), lp->getNumCols());

    // Adjusts the matrix coeficient
    // TODO(danielrocha): maybe look for the constraint in the hash
    lp->chgCoef(index, lp->getNumCols() - 1, -1.0);
  }
}

void SolverGeracaoColunas::AddIntegerSolutionColumnsWithCoeficients(
    const ProblemData& p, const ProblemSolution& int_sol,
    VariableGeracaoColunasContainer* vContainer, OPT_LP* lp) {
  //colocando solucao inteira
  for(int mac = 0; mac < p.num_machines(); ++mac) {  
    vector<short> tasks_assigned;
    int_sol.GetMachineAssignments(mac, &tasks_assigned);
    AddNewColumnWithTasks(int_sol.AssignmentCost(mac), 0.0, OPT_INF,
                          mac, tasks_assigned, &vContainer_, lp);
  }
}

void SolverGeracaoColunas::AddNewColumnWithTasks(
    double cost, double lower_bound,
    double upper_bound, int machine,
    const vector<short>& tasks,
    VariableGeracaoColunasContainer* vContainer,
    OPT_LP* lp) {
  Column column(machine, column_count_);
  AddTasksToColumnMap(column_count_, tasks);

  VariableGeracaoColunas var;
  var.set_type(VariableGeracaoColunas::COL);
  var.set_machine(machine);
  var.set_column(column);
  var.set_cost(cost);

  int column_index = lp->getNumCols();
  vContainer->push_back(var);
  lp->newCol(OPT_COL(OPT_COL::VAR_CONTINUOUS, var.cost(), lower_bound, upper_bound),
             var.ToString().c_str());
             //NULL);
             
  CHECK_EQ(vContainer->size(), lp->getNumCols());

  // TODO(danielrocha): maybe look for the constraints in the hash
  lp->chgCoef(problem_data_->num_tasks() + machine, column_index, 1.0);
  for(int task_idx = 0; task_idx < static_cast<int>(tasks.size()); ++task_idx)
    lp->chgCoef(tasks[task_idx], column_index, 1.0);

  column_count_++;
}

void SolverGeracaoColunas::AddNewColumnReuseTasks(
    const VariableGeracaoColunas& var,
    double lower_bound, double upper_bound, 
    VariableGeracaoColunasContainer* vContainer,
    OPT_LP* lp) {
  int column_index = lp->getNumCols();
  vContainer->push_back(var);
  lp->newCol(OPT_COL(OPT_COL::VAR_CONTINUOUS, var.cost(), lower_bound, upper_bound),
             var.ToString().c_str());         
             //NULL);
             
  CHECK_EQ(vContainer->size(), lp->getNumCols());

  // TODO(danielrocha): maybe look for the constraints in the hash
  lp->chgCoef(problem_data_->num_tasks() + var.column().machine(), column_index, 1.0);
  const vector<short>& tasks = GetColumnTasks(var.column().index());
  for (int task_idx = 0; task_idx < static_cast<int>(tasks.size()); ++task_idx)
    lp->chgCoef(tasks[task_idx], column_index, 1.0);
}

/********************************************************************
**                    CONSTRAINT CREATION                          **
*********************************************************************/
void SolverGeracaoColunas::AddSumAssignmentsPerTaskEqualsOneConstraints(
    const ProblemData& pd, ConstraintGeracaoColunasHash* cHash, OPT_LP* lp) {
  ConstraintGeracaoColunas cons;
  for (int task = 0; task < pd.num_tasks(); ++task) {
    cons.Clear();
    cons.set_type(ConstraintGeracaoColunas::C_SUM_ALLOCATIONS_PER_TASK_EQUALS_ONE);
    cons.set_task(task);

    if (cHash->find(cons) != cHash->end()) {
      LOG(FATAL) << "Constraint cannot be added twice " << cons.ToString();
      continue;
    }

    (*cHash)[cons] = lp->getNumRows();
    lp->newRow(OPT_ROW(OPT_ROW::EQUAL, 1.0, const_cast<char*>(cons.ToString().c_str())));
  }
}

void SolverGeracaoColunas::AddSumAssignmentsPerMachineAtMostOneConstraints(
    const ProblemData& pd, ConstraintGeracaoColunasHash* cHash, OPT_LP* lp) {
  ConstraintGeracaoColunas cons;
  for (int machine = 0; machine < pd.num_machines(); ++machine) {
    cons.Clear();
    cons.set_type(ConstraintGeracaoColunas::C_SUM_ALLOCATIONS_PER_MACHINE_AT_MOST_ONE);
    cons.set_machine(machine);

    if (cHash->find(cons) != cHash->end()) {
      LOG(FATAL) << "Constraint cannot be added twice " << cons.ToString();
      continue;
    }

    (*cHash)[cons] = lp->getNumRows();
    OPT_ROW row(0);
    row.setSense(OPT_ROW::LESS);
    row.setRhs(1.0);
    row.setName(const_cast<char*>(cons.ToString().c_str()));
    lp->newRow(row);
  }
}

/********************************************************************
**                SOLVING AND RESULTS METHODS                      **
*********************************************************************/

void SolverGeracaoColunas::GenerateSolution(
    const ProblemData& p,
    const VariableGeracaoColunasContainer& vContainer,
    const OPT_LP& lp,
    ProblemSolution* sol) const {
  // Loads the solution into the Variable hash and constructs the ProblemSolution
  vector<double> current_sol(lp.getNumCols());
  const_cast<OPT_LP&>(lp).getX(&current_sol[0]);

  vector<vector<double> > x(p.num_machines(),
                            vector<double>(p.num_tasks(), 0.0));
  GetXijMatrix(current_sol, vContainer, &x);

  sol->Clear();
  for (int machine = 0; machine < p.num_machines(); ++machine) {
    for (int task = 0; task < p.num_tasks(); ++task) {
      if (x[machine][task] > Globals::EPS())
        sol->set_assignment(task, machine);
    }
  }

  CHECK(sol->IsValid());
}

int SolverGeracaoColunas::Solve(const SolverOptions& options,
                                SolverStatus* output_status) {
  // Sets the time limits
  time_limit_in_milliseconds_ = static_cast<uint64>(options.max_time() * 1000.0);

  // Reseta e re-inicia o relogio
  stopwatch_->Reset();
  stopwatch_->Start();

  ProblemSolution* sol = &output_status->final_sol;
  if (!has_generated_initial_columns_) {
    has_generated_initial_columns_ = true;
    OPTSTAT first_status = AddInitialColumns(options.only_first_solution(),
                                             options.time_for_first_solution(), sol);
    if (first_status == OPTSTAT_MIPOPTIMAL) {
      output_status->status = OPTSTAT_MIPOPTIMAL;
    } else if (options.only_first_solution()) {
      output_status->status = OPTSTAT_FEASIBLE;
    }

    if (options.cut_off_value() > 0.0 &&
        options.cut_off_value() < sol->cost())
      sol->set_cost(static_cast<int>(options.cut_off_value() + Globals::EPS()));

    VLOG(1) << "Generated first set of columns: " << lp_->getObjVal()
            << ", cut_off value: " << sol->cost();
  }

  if (output_status->status != OPTSTAT_MIPOPTIMAL &&
      output_status->status != OPTSTAT_FEASIBLE) {
    int num_nodes_limit = 1000000;
    output_status->status = SetUpAndRunBranchAndBound(*problem_data_,
                                                      num_nodes_limit,
                                                      lp_,
                                                      sol);
  }

  // Para o relogio
  stopwatch_->Stop();

  if (TimeExpired()) {
    output_status->str_status = "time_expired";
    output_status->gap_absolute = sol->cost() - root_lower_bound_;
    output_status->gap_relative = output_status->gap_absolute / sol->cost();
  } else {
    output_status->str_status = "optimal";
    output_status->gap_absolute = output_status->gap_relative = 0.0;
  }
  output_status->node_with_best_result = node_with_best_result_;
  output_status->total_num_nodes = total_num_nodes_;

  return output_status->status;
}


/********************************************************************
**                LOCAL SEARCH CONSTRAINTS                         **
*********************************************************************/

int SolverGeracaoColunas::AddConsMaxAssignmentChanges(
    const ProblemSolution& sol, int num_changes) {
  return 0;
}

int SolverGeracaoColunas::AddConsMinAssignmentChanges(
    const ProblemSolution& sol, int num_changes) {
  return 0;
}

// Removes the constraint <cons_row> or the range [cons_row_begin, cons_row_end]
void SolverGeracaoColunas::RemoveConstraint(int cons_row) {
}
void SolverGeracaoColunas::RemoveConstraint(int cons_row_begin, int cons_row_end) {
}
  
// Changes the sense of the constraint <cons_row> and updates the RHS side to <rhs>
void SolverGeracaoColunas::ReverseConstraint(int cons_row, double rhs) {
}