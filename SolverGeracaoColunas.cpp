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
  //lp_->setMIPScreenLog(1);
  //lp_->setScreenLog(1);
  //lp_->setSimplexScreenLog(Globals::SolverLog());
  //lp_->setMIPEmphasis(0);
  lp_->setMIPRelTol(0.00);
	lp_->setMIPAbsTol(0.00);
  lp_->setParallelMode(-1);
  lp_->setAdvance(2);

  // Maximum 3 gigs, store the rest on disk (uncompressed).
  lp_->setWorkMem(2800);
  lp_->setTreLim(5000);
  lp_->setNodeFileInd(2);

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
                                               int num_tasks, short* tasks) {
  column_map_[column_index] = vector<short>(tasks, tasks + num_tasks);
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

void SolverGeracaoColunas::SetStatus(const OPTSTAT& previous, const OPTSTAT& current,
                                     OPTSTAT* final) {
  if (current == OPTSTAT_MIPOPTIMAL || previous == OPTSTAT_MIPOPTIMAL) {
    *final = OPTSTAT_MIPOPTIMAL;
    return;
  }

  // Se o atual ou o anterior são feasible, a resposta é sempre feasible.
  if (current == OPTSTAT_FEASIBLE &&
      (previous == OPTSTAT_NOINTEGER || previous == OPTSTAT_INFEASIBLE)) {
    *final = OPTSTAT_FEASIBLE;
    return;
  }
  if (previous == OPTSTAT_FEASIBLE &&
      (current == OPTSTAT_NOINTEGER || current == OPTSTAT_INFEASIBLE)) {
    *final = OPTSTAT_FEASIBLE;
    return;
  }
  if (previous == OPTSTAT_FEASIBLE && current == OPTSTAT_FEASIBLE) {
    *final = OPTSTAT_FEASIBLE;
    return;
  }

  // Se existe contradicao entre infeasible e nointeger (time expired),
  // a resposta eh nointeger.
  if ((previous == OPTSTAT_NOINTEGER && current == OPTSTAT_INFEASIBLE) ||
      (current == OPTSTAT_NOINTEGER && previous == OPTSTAT_INFEASIBLE)) {
    *final = OPTSTAT_NOINTEGER;
    return;
  }
  if (previous == OPTSTAT_NOINTEGER && current == OPTSTAT_NOINTEGER) {
    *final = OPTSTAT_NOINTEGER;
    return;
  }

  if (previous == OPTSTAT_INFEASIBLE && current == OPTSTAT_INFEASIBLE) {
    *final = OPTSTAT_INFEASIBLE;
    return;
  }

  CHECK(false) << "SetStatus: Unexpected combination, previous: " << previous
               << ", current: " << current;
}

/********************************************************************
**                   COLUMN GENERATION METHODS                     **
*********************************************************************/
void SolverGeracaoColunas::RemoveExcessColumns(
    const ProblemData& pd,
    int num_columns_to_remove,
    VariableGeracaoColunasContainer* vContainer,
    OPT_CPLEX* lp) {
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
      cost_and_index.push_back(make_pair(reduced_costs[index], index));
    }
  }

  // Ordena de forma que o maior custo fique no começo.
  sort(cost_and_index.rbegin(), cost_and_index.rend());
  VLOG(2) << "RemoveExcessColumns: ordered columns by reduced costs.";

  int removed_columns = min<int>(num_columns_to_remove, cost_and_index.size());
  vector<int> removed_indices(removed_columns);
  for (int i = 0; i < removed_columns; ++i) {
    removed_indices[i] = cost_and_index[i].second;
  }
  sort(removed_indices.begin(), removed_indices.end());

  VLOG(2) << "RemoveExcessColumns: removing excessive columns from LP and vContainer.";
  lp->delSetCols(removed_columns, &removed_indices[0]);
  for (int i = removed_columns - 1; i >= 0; --i) {
    VariableGeracaoColunasContainer::iterator removed_itr =
        (vContainer->begin() + removed_indices[i]);
    RemoveColumnTasksFromMap(removed_itr->column().index());
    vContainer->erase(removed_itr);
  }
}

double SolverGeracaoColunas::GenerateColumns(const ProblemData& p,
                                             OPT_CPLEX* lp,
                                             int* num_columns,
                                             vector<vector<short> >* fixed_vars,
                                             double best_solution_value) {

  VLOG(3) << "GenerateColumns: {num_columns: " << *num_columns
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
      status = lp->optimize(kMethodsToTry[method]);
      if (status == OPTSTAT_LPOPTIMAL || status == OPTSTAT_MIPOPTIMAL)
        break;
    }
    
    if (status != OPTSTAT_LPOPTIMAL && status != OPTSTAT_MIPOPTIMAL) {
      VLOG(1) << "GenerateColumns: infeasible.";
      return Globals::Infinity();
      //assert(false);
    }

    objective_value = lp->getObjVal();
    VLOG_EVERY_N(4, 50)
      << "GenerateColumns: loop start, objective_value: " << objective_value
      << ", status: " << status;

    if (TimeExpired()) {
      VLOG(3) << "GenerateColumns: time expired, returning " << objective_value;
      return objective_value;
    }

    // Mantem um limite no numero de colunas utilizadas: se for maior
    // que 40000, deleta 10000 colunas (escolhendo as colunas que tem o maior
    // custo reduzido) e re-otimiza.
    if(lp->getNumCols() > kMaxNumberColumns_) {
      VLOG(3) << "GenerateColumns: excessive number of columns: "
              << lp->getNumCols();
      RemoveExcessColumns(p, kNumColumnsToRemove_, &vContainer_, lp);
      lp->optimize(METHOD_PRIMAL);
    }
    
    // Os valores duais servirão para setar os preços para o subproblema
    vector<double> dual_values(lp->getNumRows(), 0.0);
    lp->getPi(&dual_values[0]);
    
    // Custos reduzidos calculados a partir da solução da mochila
    vector<double> reduced_costs(p.num_machines());

    // Precos, consumo e numero da tarefa, um elemento para cada tarefa
    // com preco positivo.
    vector<int> prices(p.num_tasks()), consume_vector(p.num_tasks());
    vector<short> used_tasks(p.num_tasks());
    // Resultado do knapsack.
    vector<int> knapsack_result(p.num_tasks());
    // Tarefas utilizadas no knapsack.
    vector<short> knapsack_tasks(p.num_tasks());

    // Reserva os vetores, para que não seja necessário re-alocalos.
    prices.reserve(p.num_tasks());
    consume_vector.reserve(p.num_tasks());
    used_tasks.reserve(p.num_tasks());
    knapsack_result.reserve(p.num_tasks());
    knapsack_tasks.reserve(p.num_tasks());

    // Tenta gerar uma coluna para cada máquina
    for (int machine = 0; machine < p.num_machines(); ++machine) {
      int used_capacity = 0;

      // Os precos de cada tarefa para o problema da mochila, ajustados
      // em relação à fixação ou não das variáveis.
      int num_usable_tasks = 0;
      for (int task = 0; task < p.num_tasks(); ++task) {
        int price = static_cast<int>(dual_values[task] * 100.0) -
                    (p.cost(machine, task) * 100);

        // Se o preco for negativo ou a alocacao maquina/tarefa ja
        // estiver fixada em zero, o preco é zero.
        if ((*fixed_vars)[machine][task] == 0)
          price = 0;

        // Se a alocacao maquina/tarefa ja estiver fixada em um,
        // o preco é zero e diminuimos a capacidade da maquina.
        if((*fixed_vars)[machine][task] == 1) {
          price = 0;
          used_capacity += p.consume(machine, task);
        }

        if (price > 0) {
          prices[num_usable_tasks] = price;
          used_tasks[num_usable_tasks] = task;
          consume_vector[num_usable_tasks] = p.consume(machine, task);
          ++num_usable_tasks;
        }
      }

      int knapsack_value = minknap(num_usable_tasks, &prices[0],
                                   const_cast<int*>(&consume_vector[0]),
                                   &knapsack_result[0],
                                   p.capacity(machine) - used_capacity);

      // Adiciona à solução encontrada as variáveis fixadas
      for(int task = 0; task < p.num_tasks(); ++task) {
        if ((*fixed_vars)[machine][task] == 1) {
          used_tasks[num_usable_tasks] = task;
          knapsack_result[num_usable_tasks] = 1;
          ++num_usable_tasks;
        }
      }

      double knapsack_price = 0.0;
      int assignment_cost = 0;
      int assignment_consume = 0;
      int num_used_tasks_in_knapsack = 0;
      for(int task_idx = 0; task_idx < num_usable_tasks; ++task_idx) {
        const short& task = used_tasks[task_idx];
        if (knapsack_result[task_idx] > 0) {
          knapsack_price += dual_values[task];
          assignment_cost += p.cost(machine, task);
          assignment_consume += p.consume(machine, task);
          knapsack_tasks[num_used_tasks_in_knapsack++] = task;
        }
      }
      knapsack_price += dual_values[p.num_tasks() + machine];

      reduced_costs[machine] = assignment_cost - knapsack_price;

      // Se a coluna gerada tem custo reduzido negativo, adiciona à solução
      if(reduced_costs[machine] < -Globals::EPS()) {
        CHECK_LE(assignment_consume, p.capacity(machine))
          << "The knapsack solution consumes more than the machine capacity!";

        generated_and_added_column = true;
        // Adiciona a nova coluna, com valor na função objetivo igual ao custo
        // da resposta da mochila.
        AddNewColumnWithTasks(assignment_cost, 0.0, OPT_INF, machine,
                              num_used_tasks_in_knapsack, &knapsack_tasks[0],
                              &vContainer_, lp);
        *num_columns += 1;

        VLOG_EVERY_N(4, 197)
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
      if (max_lower_bound < objective_value + sum_reduced_costs)
        max_lower_bound = objective_value + sum_reduced_costs;

      // Se o lower bound encontrado for maior que o upper bound, pode retornar
      //if (objective_value + sum_reduced_costs > best_solution_value - 1.0 + Globals::BigEPS()) {
      if (objective_value + sum_reduced_costs > best_solution_value) {
        VLOG(4) << "GenerateColumns: end, obj_val + reduced_costs >= best solution: "
                << objective_value << " + " << sum_reduced_costs << " >= "
                << best_solution_value - 1.0 + Globals::BigEPS();
        return objective_value + sum_reduced_costs;
      }

      VLOG_EVERY_N(4, 50)
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
    const VariableGeracaoColunasContainer& vContainer, OPT_CPLEX* lp) {
  VLOG(2) << "Adjusting stabilization bounds to " << bound;
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
    VariableGeracaoColunasContainer* vContainer, OPT_CPLEX* lp) {
  VLOG(2) << "Updating stabilization costs";
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
    const ProblemData& p, OPT_CPLEX* lp, int* num_columns,
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
      CHECK_GE(new_lower_bound + Globals::BigEPS(), lower_bound - Globals::BigEPS())
        << "Stabilization: new_lower_bound (" << new_lower_bound << ") < "
        << "lower_bound (" << lower_bound << ")";

    // O 'max' abaixo só é necessário devido a problemas de precisão numérica
    lower_bound = max<double>(lower_bound, new_lower_bound);
    //if (lower_bound >= best_solution_value - 1.0 + Globals::BigEPS()) {
    if (lower_bound > best_solution_value) {
      VLOG(2) << "Stabilization: lower_bound > best_solution_value: "
              << lower_bound << " > " << best_solution_value;
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
      VLOG(1) << "Stabilization: time expired, returning " << lower_bound;
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
    double cut_off_value,
    bool first_only,
    OPT_CPLEX* lp,
    ProblemSolution* integer_solution) {
  int num_columns = 0;
  double pct_tree_solved = 0.0;
  int num_visited_nodes = 0;

  // Cria uma matriz de variáveis fixadas, deixando inicialmente todas não fixadas
  vector<vector<short> > fixed_vars(p.num_machines(),
                                    vector<short>(p.num_tasks(), -1));

  OPTSTAT status = OPTSTAT_INFEASIBLE;
  BB(p,
     num_nodes_limit,  // limiteNos = 1000000
     0,  // profundidade = 0
     0,  // Lower bound = 0
     cut_off_value,  // upper bound = cut_off_value
     first_only,  // first_only = first_only
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
                                double cut_off_value,
                                bool first_only,
                                OPT_CPLEX* lp,
                                ProblemSolution* best_solution,
                                vector<vector<short> >* fixed_vars,
                                int* num_columns,
                                double* pct_tree_solved,
                                int* num_visited_nodes,
                                OPTSTAT* status) {
  //tLpNo.reset();
  //tMochilaNo.reset();

  if (first_only && *status == OPTSTAT_FEASIBLE) {
    VLOG(2) << "BB(" << depth << "): returning first solution only, cost: "
            << best_solution->cost();
    return Globals::Infinity();
  }

  VLOG(2) << "BB(" << depth << "): {num_visited_nodes: " << *num_visited_nodes
          << ", pct_tree_solved: " << *pct_tree_solved << ", lower_bound: "
          << lower_bound << "}";
  double new_lower_bound;
  if (stabilize_column_generation_) {
    new_lower_bound = GenerateColumnsWithStabilization(
        p, lp, num_columns, fixed_vars, cut_off_value);
  } else {
    new_lower_bound = GenerateColumns(
        p, lp, num_columns, fixed_vars, cut_off_value);
  }
  VLOG(2) << "BB(" << depth << "): column generation lower bound: "
          << new_lower_bound;
  if (depth == 0) {
    VLOG(1) << "BB(" << depth << "): root node lower bound: " << new_lower_bound;
    root_lower_bound_ = new_lower_bound;
  } else {
    ++total_num_nodes_;
  }

  // Se o melhor LB encontrado for maior que a melhor solução encontrada
  if (new_lower_bound >= cut_off_value - 1.0 + Globals::BigEPS() ||
      new_lower_bound == Globals::Infinity()) {
  //if (new_lower_bound > cut_off_value) {
    *pct_tree_solved += pow(0.5, static_cast<double>(depth));
    if (*pct_tree_solved >= 1.0 - Globals::EPS()) {
      // Se a porcentagem do BB já resolvido for >= 1 ajusta resposta
      SetStatus(*status, OPTSTAT_MIPOPTIMAL, status);
    } else {
      SetStatus(*status, OPTSTAT_INFEASIBLE, status);
    }
    VLOG(1) << "BB(" << depth << "): lower_bound > best_solution, pct_tree_solved: "
            << *pct_tree_solved << ", status = " << *status;
    return new_lower_bound;
  }

  *num_visited_nodes += 1;
  if (*num_visited_nodes >= num_nodes_limit) {
    SetStatus(*status, OPTSTAT_NOINTEGER, status);
    VLOG(1) << "BB(" << depth << "): num_visited_nodes > limit: "
            << *num_visited_nodes << ", return: " << new_lower_bound
            << ", status: " << *status;
    return new_lower_bound;
  }

  if (TimeExpired()) {
    SetStatus(*status, OPTSTAT_NOINTEGER, status);
    VLOG(1) << "BB(" << depth << "): time expired (after GenerateColumns), returning "
            << new_lower_bound << ", status: " << *status;
    return new_lower_bound;
  }

  CHECK_GE(new_lower_bound + Globals::EPS(), lower_bound)
    << "The column generation lower bound must be at least as good as the input "
    << "bound.";

  vector<double> current_solution(lp->getNumCols());
  lp->getX(&current_solution[0]);

  vector<vector<double> > x(p.num_machines(),
                            vector<double>(p.num_tasks(), 0.0));
  GetXijMatrix(current_solution, vContainer_, &x);
  VLOG(1) << "BB(" << depth << "): got current solution and generated Xij matrix.";
  if (rins_period_ > 0 && (depth % 2 == 0 || depth >= 18)) {
    int TL = max<int>(100, 1000 / static_cast<int>(pow(1.3, depth + 1)));
    VLOG(1) << "BB(" << depth << "): calling RINS with TL = " << TL;
    if (RINS(x, best_solution, TL)) {
      node_with_best_result_ = total_num_nodes_;
      time_to_best_result_ = stopwatch_->ElapsedMilliseconds / 1000.0;
    }
  }

  // TODO(danielrocha): move parameters to a better place
  static const int kMaxDepthFixingVars_ = 4;
  static const int kNumberOfLookupsFixingVars_[] = { 6, 4, 2, 1 };
  //static const int kNumberOfLookupsFixingVars_[] = { 15, 5, 3, 2 };
  //static const int kNumberOfLookupsFixingVars_[] = { 4, 3, 2, 1 };

  int num_integer_vars = 0;
  int num_variables_on_one = 0;
  double current_cost = 0.0;
  int fixed_machine = -1;
  int fixed_task = -1;
  if(depth < kMaxDepthFixingVars_) {
    VLOG(2) << "BB(" << depth << "): calling SelectFixedVariables to select variables.";
    num_integer_vars =
      SelectFixedVariable(p, kNumberOfLookupsFixingVars_[depth],
                          x, cut_off_value, lp, best_solution, fixed_vars, num_columns,
                          &fixed_machine, &fixed_task);
  } else {
    VLOG(2) << "BB(" << depth << "): selecting variables using randomized heuristic.";
    set<FixingCandidate> candidates;
    for (int machine = 0; machine < p.num_machines(); ++machine)
      for (int task = 0; task < p.num_tasks(); ++task)
        if (x[machine][task] > 1.0 - Globals::EPS() ||
            x[machine][task] < Globals::EPS()) {
          ++num_integer_vars;
          if (x[machine][task] > 1.0 - Globals::EPS()) {
            ++num_variables_on_one;
            current_cost += p.cost(machine, task);
          }
        } else {
          FixingCandidate cand;
          cand.machine = machine;
          cand.task = task;
          cand.value = EvaluateVariableToFix(x[machine][task]);
          candidates.insert(cand);
          if (candidates.size() > 10)
            candidates.erase(candidates.begin());
        }
    // Randomly selects from the candidates
    if (candidates.size() > 0) {
      vector<FixingCandidate> v(candidates.begin(), candidates.end());
      int c = Globals::rg()->IRandom(0, candidates.size() - 1);
      fixed_machine = v[c].machine;
      fixed_task = v[c].task;
    }
  }
  VLOG(1) << "BB(" << depth << "): fixed task: " << fixed_task << " and machine: "
          << fixed_machine;

  // Se a solução ótima do problema linear é composta de variáveis inteiras, ela é
  // também o ótimo para o problema inteiro. Não é mais necessário fazer B&B.
  if (num_integer_vars == p.num_machines() * p.num_tasks()) {
    *pct_tree_solved += pow(0.5, static_cast<double>(depth));
    bool vns_feasible = VnsCutUtil::IsValidFinalXijMatrix(vns_cuts_, x);
    bool ellipsoidal_feasible =
        EllipsoidalCutUtil::IsValidFinalXijMatrix(ellipsoidal_cuts_, x);

    if (vns_feasible && ellipsoidal_feasible) {
      SetStatus(*status, OPTSTAT_FEASIBLE, status);
      if (best_solution->cost() == 0 || lp->getObjVal() < best_solution->cost()) {
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
        time_to_best_result_ = stopwatch_->ElapsedMilliseconds / 1000.0;
      }
      
      // Se temos todas as variáveis inteiras e já resolvemos toda a árvore, 
      // temos a solução ótima.
      if(*pct_tree_solved > 1.0 - Globals::EPS()) {
        SetStatus(*status, OPTSTAT_MIPOPTIMAL, status);
      }
      VLOG(1) << "BB(" << depth << "): all integer: {pct_tree_solved: "
              << *pct_tree_solved << ", status: " << *status << "}";
      return new_lower_bound;
    } else if (!vns_feasible) {
      SetStatus(*status, OPTSTAT_INFEASIBLE, status);
      VLOG(1) << "BB(" << depth << "): all integer: {pct_tree_solved: "
              << *pct_tree_solved << ", status: infeasible due to VNS cuts, "
              << ", final status: " << *status << "}.";
      return Globals::Infinity();
    } else if (!vns_feasible) {
      SetStatus(*status, OPTSTAT_INFEASIBLE, status);
      VLOG(1) << "BB(" << depth << "): all integer: {pct_tree_solved: "
              << *pct_tree_solved << ", status: infeasible due to Ellipsoidal cuts, "
              << ", final status: " << *status << "}.";
      return Globals::Infinity();
    }
  }
  /* else {
    // Calcula uma aproximacao do quanto resta adicionar ao custo da solucao
    if ((p.num_tasks() - num_variables_on_one) * p.MinimumAssignmentCost() + current_cost >
         best_solution->cost()) {
      VLOG(1) << "BB(" << depth << "): estimation shows that this node "
              << "is not promising: num_variables_on_one: " << num_variables_on_one
              << ", current_cost: " << current_cost << ", minimum assignment cost: "
              << p.MinimumAssignmentCost() << ", best solution: "
              << best_solution->cost();
      return new_lower_bound;
    }
  }*/

  if (fixed_machine < 0 || fixed_task < 0) {
    SetStatus(*status, OPTSTAT_INFEASIBLE, status);
    VLOG(1) << "BB(" << depth << "): couldn't select any fixing variable, "
            << "returning status: " << *status;
    return Globals::Infinity();
  }

  double new_cut_off_value = cut_off_value;
  if (best_solution->cost() > 0 && best_solution->cost() < new_cut_off_value)
    new_cut_off_value = best_solution->cost();

  VLOG(2) << "BB(" << depth << "): number of integer variables: "
          << num_integer_vars << ", new cutoff: " << new_cut_off_value;

  FixingSense order[2] = { FIX_ON_ONE, FIX_ON_ZERO };
  if (best_solution->cost() > 0 && best_solution->assignment(fixed_task) != fixed_machine) {
    order[0] = FIX_ON_ZERO; order[1] = FIX_ON_ONE;
  }

  if (VnsCutUtil::CanFixCandidate(vns_cuts_, *fixed_vars, fixed_machine,
                                  fixed_task, order[0])) {
    FixVariableAndContinueBB(order[0],
                             fixed_machine,
                             fixed_task,
                             num_nodes_limit,
                             new_lower_bound,
                             new_cut_off_value,
                             first_only,
                             p,
                             lp,
                             best_solution,
                             fixed_vars,
                             num_columns,
                             depth,
                             pct_tree_solved,
                             num_visited_nodes,
                             status);
  } else {
    *pct_tree_solved += pow(0.5, static_cast<double>(depth + 1));
    VLOG(1) << "BB(" << depth << "): unable to fix var due to VNS cuts, task: "
            << fixed_task << ", machine: " << fixed_machine << ", sense: " << FIX_ON_ONE
            << ", pct tree: " << *pct_tree_solved;
  }

  if (TimeExpired()) {
    SetStatus(*status, OPTSTAT_NOINTEGER, status);
    VLOG(1) << "BB(" << depth << "): time expired (after FixAndContinue), returning "
            << new_lower_bound << ", status: " << *status;
    return new_lower_bound;
  }

  if (VnsCutUtil::CanFixCandidate(vns_cuts_, *fixed_vars, fixed_machine,
                                  fixed_task, order[1])) {
    FixVariableAndContinueBB(order[1],
                             fixed_machine,
                             fixed_task,
                             num_nodes_limit,
                             new_lower_bound,
                             new_cut_off_value,
                             first_only,
                             p,
                             lp,
                             best_solution,
                             fixed_vars,
                             num_columns,
                             depth,
                             pct_tree_solved,
                             num_visited_nodes,
                             status);
  } else {
    *pct_tree_solved += pow(0.5, static_cast<double>(depth + 1));
    VLOG(1) << "BB(" << depth << "): unable to fix var due to VNS cuts, task: "
            << fixed_task << ", machine: " << fixed_machine << ", sense: " << FIX_ON_ZERO
            << ", pct tree: " << *pct_tree_solved;
  }

  if (*pct_tree_solved >= 1.0 - Globals::EPS()) {
    if (*status == OPTSTAT_FEASIBLE && !first_only)
      *status = OPTSTAT_MIPOPTIMAL;
    VLOG(1) << "BB(" << depth << "): 100% of the tree solved, status: " << *status;
  }

  VLOG(1) << "BB(" << depth << "): end, return value: " << new_lower_bound
          << ", status: " << *status;
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
    int num_nodes_limit, double lower_bound, double cut_off_value,
    bool first_only, const ProblemData& p, OPT_CPLEX* lp,
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
  double retorno = BB(p, num_nodes_limit, depth + 1, lower_bound, cut_off_value,
                      first_only, lp, best_solution, fixed_vars,
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
    const vector<vector<double> >& x, double cut_off, OPT_CPLEX* lp,
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
                               1000000, cut_off, false, p, lp, best_solution, fixed_vars,
                               num_columns, 0, &pct_tree_solved,
                               &num_visited_nodes, &status);

    num_visited_nodes = 0;
    pct_tree_solved = 0.0;
    double fixed_on_one =
      FixVariableAndContinueBB(FIX_ON_ONE, candidate.machine, candidate.task, 1,
                               1000000, cut_off, false, p, lp, best_solution, fixed_vars,
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
      VLOG(1) << "SelectFixedVariable: time expired, returning: fixed_machine: "
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
  LOG(INFO) << "Got first integer solution, with cost " << integer_solution->cost()
            << ", status: " << first_status;

  // Adiciona a solução inteira ao modelo
  AddIntegerSolutionColumnsWithCoeficients(*problem_data_,
                                           *integer_solution, &vContainer_, lp_);
  lp_->optimize(METHOD_AUTOMATIC);

  return first_status;
}

void SolverGeracaoColunas::AddStabilizationColumnsWithCoeficients(
    const ProblemData& p, const vector<double>& relaxed_dual_values,
    VariableGeracaoColunasContainer* vContainer, OPT_CPLEX* lp) {
  const double initial_upper_bound = 0.0;

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
    vContainer->push_back(var);
    double lower_bound = 0.0, upper_bound = initial_upper_bound;
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
    vContainer->push_back(var);
    double lower_bound = 0.0, upper_bound = initial_upper_bound;
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
    VariableGeracaoColunasContainer* vContainer, OPT_CPLEX* lp) {
  //colocando solucao inteira
  for(int mac = 0; mac < p.num_machines(); ++mac) {  
    vector<short> tasks_assigned;
    int_sol.GetMachineAssignments(mac, &tasks_assigned);
    AddNewColumnWithTasks(int_sol.AssignmentCost(mac), 0.0, OPT_INF,
                          mac, tasks_assigned.size(), &tasks_assigned[0],
                          &vContainer_, lp);
  }
}

void SolverGeracaoColunas::AddNewColumnWithTasks(
    double cost, double lower_bound,
    double upper_bound, int machine,
    int num_tasks, short* tasks,
    VariableGeracaoColunasContainer* vContainer,
    OPT_CPLEX* lp) {
  Column column(machine, column_count_);
  AddTasksToColumnMap(column_count_, num_tasks, tasks);

  VariableGeracaoColunas var;
  var.set_type(VariableGeracaoColunas::COL);
  var.set_machine(machine);
  var.set_column(column);
  var.set_cost(cost);

  int column_index = lp->getNumCols();
  vContainer->push_back(var);
  lp->newCol(OPT_COL(OPT_COL::VAR_CONTINUOUS, var.cost(), lower_bound, upper_bound),
             var.ToString().c_str());
             
  CHECK_EQ(vContainer->size(), lp->getNumCols());

  // TODO(danielrocha): maybe look for the constraints in the hash
  lp->chgCoef(problem_data_->num_tasks() + machine, column_index, 1.0);
  for(int task_idx = 0; task_idx < num_tasks; ++task_idx)
    lp->chgCoef(tasks[task_idx], column_index, 1.0);

  column_count_++;
}

void SolverGeracaoColunas::AddNewColumnReuseTasks(
    const VariableGeracaoColunas& var,
    double lower_bound, double upper_bound, 
    VariableGeracaoColunasContainer* vContainer,
    OPT_CPLEX* lp) {
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
    const ProblemData& pd, ConstraintGeracaoColunasHash* cHash, OPT_CPLEX* lp) {
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
    const ProblemData& pd, ConstraintGeracaoColunasHash* cHash, OPT_CPLEX* lp) {
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

void SolverGeracaoColunas::GenerateSolution(ProblemSolution* sol) {
  GenerateSolution(*problem_data_, vContainer_, *lp_, sol);
}

void SolverGeracaoColunas::GenerateSolution(
    const ProblemData& p,
    const VariableGeracaoColunasContainer& vContainer,
    const OPT_CPLEX& lp,
    ProblemSolution* sol) const {
  // Loads the solution into the Variable hash and constructs the ProblemSolution
  vector<double> current_sol(lp.getNumCols());
  const_cast<OPT_CPLEX&>(lp).getX(&current_sol[0]);

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
  CHECK_NOTNULL(output_status);
  // Sets the time limits
  time_limit_in_milliseconds_ = static_cast<uint64>(options.max_time() * 1000.0);
  rins_period_ = options.rins_period();

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

    VLOG(1) << "Generated first set of columns: " << lp_->getObjVal();
  }

  VLOG(1) << "Cut_off value: " << options.cut_off_value() << ", first sol: " << sol->cost();

  if ((!options.only_first_solution() || vns_cuts_.size() > 0) &&
       output_status->status != OPTSTAT_MIPOPTIMAL) {
    int num_nodes_limit = 1000000;
    output_status->status = SetUpAndRunBranchAndBound(*problem_data_,
                                                      num_nodes_limit,
                                                      options.cut_off_value(),
                                                      options.only_first_solution(),
                                                      lp_,
                                                      sol);
  }

  // Para o relogio
  stopwatch_->Stop();

  if (TimeExpired() || output_status->status == OPTSTAT_FEASIBLE) {
    output_status->str_status = (TimeExpired() ? "time_expired" : "feasible");
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
  if (vns_cuts_.size() > 0 &&
      vns_cuts_[vns_cuts_.size() - 1].solution() == sol) {
    vns_cuts_[vns_cuts_.size() - 1].set_max_changes(num_changes);
  } else {
    VnsCut cut;
    cut.set_max_changes(num_changes);
    cut.set_solution(sol);
    vns_cuts_.push_back(cut);
  }

  return vns_cuts_.size() - 1;
}

int SolverGeracaoColunas::AddConsMinAssignmentChanges(
    const ProblemSolution& sol, int num_changes) {
  VnsCut cut;
  cut.set_min_changes(num_changes);
  cut.set_solution(sol);
  vns_cuts_.push_back(cut);
  return vns_cuts_.size() - 1;
}

// Removes the constraint <cons_row> or the range [cons_row_begin, cons_row_end]
void SolverGeracaoColunas::RemoveConstraint(int cons_row) {
  CHECK_GE(cons_row, 0);
  CHECK_LT(cons_row, vns_cuts_.size());
  vns_cuts_.erase(vns_cuts_.begin() + cons_row);
}

void SolverGeracaoColunas::RemoveConstraint(int cons_row_begin, int cons_row_end) {
  CHECK_GE(cons_row_begin, 0);
  CHECK_LE(cons_row_begin, cons_row_end);
  CHECK_LT(cons_row_end, vns_cuts_.size());
  vns_cuts_.erase(vns_cuts_.begin() + cons_row_begin, vns_cuts_.begin() + cons_row_end + 1);
}

// Changes the sense of the constraint <cons_row> and updates the RHS side to <rhs>
void SolverGeracaoColunas::ReverseConstraint(int cons_row, int rhs) {
  VnsCut* cut = &vns_cuts_[cons_row];
  if (cut->min_changes() >= 0) {
    cut->set_max_changes(cut->min_changes());
    cut->set_min_changes(-1);
  } else {
    cut->set_min_changes(cut->max_changes());
    cut->set_max_changes(-1);
  }
}

int SolverGeracaoColunas::AddEllipsoidalConstraint(
    const vector<const ProblemSolution*>& x, OPT_ROW::ROWSENSE constraint_sense, int F) {
  ellipsoidal_cuts_.push_back(EllipsoidalCut(x, constraint_sense, F));
  return ellipsoidal_cuts_.size() - 1;
}

bool SolverGeracaoColunas::RINS(const vector<vector<double> >& x,
                                ProblemSolution* sol, int time_limit) {
  SolverFormulacaoPadrao solver(Globals::instance());
  SolverOptions options;
  //options.set_emphasis_on_feasibility(true);
  solver.Init(options);

  for (int task = 0; task < problem_data_->num_tasks(); ++task) {
    for (int machine = 0; machine < problem_data_->num_machines(); ++machine) {
      if (x[machine][task] < Globals::BigEPS() && sol->assignment(task) != machine) {
        // Fix on zero
        solver.FixVar(machine, task, false);
      } else if (x[machine][task] > 1.0 - Globals::BigEPS() &&
                 sol->assignment(task) == machine) {
        // Fix on one
        solver.FixVar(machine, task, true);
      }
    }
  }

  options.set_max_time(time_limit);
  options.set_cut_off_value(sol->cost());
  SolverStatus status;
  solver.Solve(options, &status);
  if (status.status == OPTSTAT_MIPOPTIMAL || status.status == OPTSTAT_FEASIBLE) {
    VLOG(1) << "RINS: generated improved solution, status: " << status.status
            << ", cost: " << status.final_sol.cost();
    *sol = status.final_sol;
    return true;
  } else {
    VLOG(1) << "RINS: failed, status: " << status.status;
    return false;
  }
}