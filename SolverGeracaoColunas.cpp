#include "SolverGeracaoColunas.h"

#include <cassert>

#include <algorithm>
#include <iostream>
#include <numeric>
#include <set>


#include "opt_cplex.h"
#include "minknap.c"
#include "SolverFormulacaoPadrao.h"
#include "ProblemData.h"

SolverGeracaoColunas::SolverGeracaoColunas(ProblemData* problem_data) :
  Solver(problem_data),
  column_count_(0) {
}

SolverGeracaoColunas::~SolverGeracaoColunas() {
    if (lp_ != NULL)
        delete lp_;
}


/********************************************************************
**                       UTILITY METHODS                           **
*********************************************************************/
void SolverGeracaoColunas::SetUpCplexParams(OPT_LP* lp) {
  lp->setMIPScreenLog(0);
  lp->setMIPEmphasis(0);
  lp->setMIPRelTol(0.00);
	lp->setMIPAbsTol(0.00);
}

// Calcula a matriz Xij com os valores fracionarios das
// associacoes de tarefas à maquinas. Em outras palavras, transforma
// a formulação de geração em colunas na formulação padrão calculando
// as variáveis binárias Xij.
void SolverGeracaoColunas::GetXijMatrix(const vector<double>& current_sol,
                                        const VariableGeracaoColunasHash& vHash,
                                        vector<vector<double> >* x) const {
  for (VariableGeracaoColunasHash::const_iterator it = vHash.begin();
       it != vHash.end(); ++it) {
    const VariableGeracaoColunas& var = it->first;
    int index = it->second;
    // Se a coluna i está sendo usada (mesmo que fracionalmente)
    if (var.type() == VariableGeracaoColunas::COL &&
        current_sol.at(index) > Globals::EPS()) {
      int machine = var.column().machine();
      // TODO(danielrocha): CHECKs
      assert(lp_->getCoef(problem_data_->num_tasks() + machine, index) >
             1.0 - Globals::EPS());
      assert(machine >= 0);

      // Adiciona a fração que está sendo usada dessa alocação à Xij
      const vector<int>& tasks = var.column().tasks();
      for (int task_idx = 0; task_idx < static_cast<int>(tasks.size()); ++task_idx) {
        (*x)[machine][tasks[task_idx]] += current_sol[index];
        // TODO(danielrocha): CHECKs
        assert(lp_->getCoef(tasks[task_idx], index) > 1 - Globals::EPS());
      }
    }
  }
}

/********************************************************************
**                   COLUMN GENERATION METHODS                     **
*********************************************************************/
void SolverGeracaoColunas::RemoveExcessColumns(const ProblemData& pd,
                                               int num_columns_to_remove,
                                               VariableGeracaoColunasHash* vHash,
                                               OPT_LP* lp) {
  vector<double> reduced_costs(lp->getNumCols(), 0.0);
  lp->getDj(&reduced_costs[0]);

  vector<pair<double, int> > cost_and_index;
  for (VariableGeracaoColunasHash::const_iterator it = vHash->begin();
       it != vHash->end(); ++it) {
    const VariableGeracaoColunas& var = it->first;
    // Só queremos remover as colunas geradas
    if (var.type() == VariableGeracaoColunas::COL) {
      int index = it->second;
      double cost = reduced_costs.at(index);
      cost_and_index.push_back(make_pair(cost, index));
      // TODO(danielrocha): add some checks that this is the right index
    }
  }

  // Ordena de forma que o maior custo fique no começo.
  sort(cost_and_index.rbegin(), cost_and_index.rend());

  int removed_columns = min<int>(num_columns_to_remove, cost_and_index.size());
  vector<int> removed_indices(removed_columns);
  for (int i = 0; i < removed_columns; ++i) {
    removed_indices[i] = cost_and_index[i].second;
  }

  lp->delSetCols(removed_columns, &removed_indices[0]);
}

double SolverGeracaoColunas::GenerateColumns(const ProblemData& p,
                                             OPT_LP* lp,
                                             int* num_columns,
                                             vector<vector<short> >* fixed_vars,
                                             double best_solution_value) {
  double max_lower_bound = 0;
  double objective_value = 0.0;

  // Em cada laço desse loop tentaremos gerar num_machines() colunas, uma para
  // cada máquina.
  bool generated_and_added_column = true;
  while(generated_and_added_column) {
    generated_and_added_column = false;

    lp->optimize(METHOD_PRIMAL);
    num_pivo_ += lp->getItCnt();
    objective_value = lp->getObjVal();

    // Mantem um limite no numero de colunas utilizadas: se for maior
    // que 40000, deleta 10000 colunas (escolhendo as colunas que tem o maior
    // custo reduzido) e re-otimiza.
    if(lp->getNumCols() > kMaxNumberColumns_) {
      RemoveExcessColumns(p, kNumColumnsToRemove_, &vHash_, lp);
      /*tLp.start();
      tLpNo.start();*/
      lp->optimize(METHOD_PRIMAL);
      num_pivo_ += lp->getItCnt();
      /*tLpNo.stop();
      tLp.stop();*/
    }
    
    // Os valores duais servirão para setar os preços para o subproblema
    vector<double> dual_values(lp->getNumRows());
    lp->getPi(&dual_values[0]);

    int num_exact_knapsacks = 0;
    
    // Custos reduzidos calculados a partir da solução da mochila
    vector<double> reduced_costs(p.num_machines());

    // Tenta gerar uma coluna para cada máquina
    for (int machine = 0; machine < p.num_machines(); ++machine) {
      int used_capacity = 0;

      // Os precos de cada tarefa para o problema da mochila, ajustados
      // em relação à fixação ou não das variáveis.
      vector<int> prices(p.num_tasks());
      for (int task = 0; task < p.num_tasks(); ++task) {
        prices[task] = (static_cast<int>(dual_values[task] * 100.0) -
                        (p.cost(machine, task) * 100));

        // Se o preco for negativo ou a alocacao maquina/tarefa ja
        // estiver fixada em zero, o preco é zero.
        if(prices[task] < 0 || (*fixed_vars)[machine][task] == 0)
          prices[task] = 0;

        // Se a alocacao maquina/tarefa ja estiver fixada em um,
        // o preco é zero e armazenamos diminuimos a capacidade da maquina.
        if((*fixed_vars)[machine][task] == 1) {
          prices[task] = 0;
          used_capacity += p.consume(machine, task);
        }
      }

      ++num_exact_knapsacks;
      /*tMochila.start();
      tMochilaNo.start();*/
      vector<int> knapsack_result(p.num_tasks());
      int knapsack_value = minknap(p.num_tasks(), &prices[0],
                                   const_cast<int*>(p.GetConsumeVector(machine)),
                                   &knapsack_result[0],
                                   p.capacity(machine) - used_capacity);
      /*tMochilaNo.stop();
      tMochila.stop();
      mExata_++;*/

      // Adiciona à solução encontrada as variáveis fixadas
      for(int task = 0; task < p.num_tasks(); ++task) {
        if (prices[task] == 0 ||(*fixed_vars)[machine][task] == 0)
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
        // TODO(danielrocha): CHECK
        assert(p.AssignmentConsume(machine, knapsack_result) <= p.capacity(machine));
        generated_and_added_column = true;

        // Adiciona a nova coluna, com valor na função objetivo igual ao custo
        // da resposta da mochila.
        vector<int> knapsack_tasks;
        for (int task = 0; task < p.num_tasks(); ++task)
          if (knapsack_result[task] > 0)
            knapsack_tasks.push_back(task);
        AddNewColumn(p.AssignmentCost(machine, knapsack_result), 0.0, OPT_INF,
                     machine, knapsack_tasks, &vHash_, lp);
        *num_columns += 1;
      }
    }
    
    if(generated_and_added_column) {
      double sum_reduced_costs = accumulate(reduced_costs.begin(),
                                            reduced_costs.end(), 0.0);

      // Se gerou coluna para todos as máquinas e encontrou um lower bound
      // melhor (maior), guarda o lower bound
      // TODO(danielrocha): porque a primeira restricao!?
      if(num_exact_knapsacks == p.num_machines() &&
         max_lower_bound < objective_value + sum_reduced_costs)
        max_lower_bound = objective_value + sum_reduced_costs;

      // Se o lower bound encontrado for maior que o upper bound, pode retornar
      // TODO(danielrocha): porque a primeira restricao!?
      if(num_exact_knapsacks == p.num_machines() &&
         objective_value + sum_reduced_costs >=
         best_solution_value - 1 + Globals::BigEPS()) {
        return objective_value + sum_reduced_costs;
      }
      /*printf("%8d %9.3lf ",cont,objective_value);
        for(int mac = 0; mac < p.num_machines(); ++mac)
          printf(" %9.3lf",reduced_costs[i]);
        printf(" %9.3lf\n",max_lower_bound);*/
    }
  }

  return objective_value;
}

double SolverGeracaoColunas::GenerateColumnsWithStabilization(
    const ProblemData& p, OPT_LP* lp, int* num_columns,
    vector<vector<short> >* fixed_vars, double best_solution_value) {
  // TODO(danielrocha): add the stabilization here.
  return GenerateColumns(p, lp, num_columns, fixed_vars, best_solution_value);
}


/********************************************************************
**                    BRANCH AND BOUND METHODS                     **
*********************************************************************/

OPTSTAT SolverGeracaoColunas::SetUpAndRunBranchAndBound(const ProblemData& p,
                                                        int num_nodes_limit,
                                                        OPT_LP* lp,
                                                        ProblemSolution* integer_solution) {
  int num_columns = 0;
  double pct_tree_solved = 0.0;
  int num_visited_nodes = 0;

  // Cria uma matriz de variáveis fixadas, deixando inicialmente todas não fixadas
  vector<vector<short> > fixed_vars(p.num_machines(),
                                    vector<short>(-1, p.num_tasks()));

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
  num_pivo_ = 0;

  double new_lower_bound =
    GenerateColumnsWithStabilization(p, lp, num_columns,
                                     fixed_vars, best_solution->cost());
  
  *num_visited_nodes += 1;
  if(*num_visited_nodes >= num_nodes_limit)
    return new_lower_bound;
  
#ifdef MENSAGEM_BB 
  printf("Nos Visitados: %5d LB: %6.3lf profundidade: %d\n",nosVisitados, LB, profundidade);
  printf("Tempo LP: %10.5lf Tempo Mochila %10.5lf Pivos: %10d\n",tLpNo.getCPUTotalSecs(), tMochilaNo.getCPUTotalSecs(),numPivo_); 
#endif

  // Se o melhor LB encontrado for maior que a melhor solução encontrada
  if(new_lower_bound >= best_solution->cost() - 1.0 + Globals::BigEPS()) {
    *pct_tree_solved += pow(0.5, static_cast<double>(depth));
    //printf("Total da Arvore Resolvida: %lf\n--------------------------------\n",acumulador);
    if(*pct_tree_solved >= 1.0 - Globals::EPS()) {
      // Se a porcentagem do BB já resolvido for >= 1 ajusta resposta
      if(*status == OPTSTAT_NOINTEGER)
        *status = OPTSTAT_INFEASIBLE;
      else if(*status == OPTSTAT_FEASIBLE)
        *status = OPTSTAT_MIPOPTIMAL;
    }

    return new_lower_bound;
  }

  // O lower bound encontrado nesse nó _tem_ que ser pelo menos tão bom quanto o anterior.
  assert(new_lower_bound + Globals::EPS() >= lower_bound);

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
  GetXijMatrix(current_solution, vHash_, &x);

  // Calcula o custo da solução fracionária representada por Xij
  /*double relaxed_solution_cost = 0.0;
  for (int machine = 0; machine < p.num_machines(); ++machine)
    for (int task = 0; task < p.num_tasks(); ++task)
      relaxed_solution_cost += p.cost(machine, task) * x[machine][task];
  */
  
  //determinando quem serah fixado
  int num_integer_vars = 0;

  // TODO(danielrocha): move parameters to a better place
  static const int kMaxDepthFixingVars_ = 4;
  static const int kNumberOfLookupsFixingVars_[] = { 10, 8, 6, 4 };

  int fixed_machine;
  int fixed_task;
  if(depth < kMaxDepthFixingVars_) {
    num_integer_vars =
      SelectFixedVariable(p, kNumberOfLookupsFixingVars_[depth],
                          x, lp, best_solution, fixed_vars, num_columns,
                          &fixed_machine, &fixed_task);
  } else {
    double bestVal = 0.0;
    for (int machine = 0; machine < p.num_machines(); ++machine)
      for (int task = 0; task < p.num_tasks(); ++task)
        if(x[machine][task] > 1.0 - Globals::EPS() ||
           x[machine][task] < Globals::EPS()) {
          num_integer_vars++;
        } else {
          double heuristic_eval = EvaluateVariableToFix(x[machine][task]);
          if (bestVal < heuristic_eval) {
            bestVal = heuristic_eval;
            fixed_machine = machine;
            fixed_task = task;
          }
        }
  }

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
      // TODO(danielrocha): CHECK below
      assert(best_solution->cost() == static_cast<int>(lp->getObjVal() + Globals::EPS()));
      //printf("!!!!!!!!!!!!!!! Melhor Solucao Inteira: %lf \n",best_solution_value);
    }

    /*tGeral.stop();
    fprintf(logSaida,"\nVal Solucao: %lf  tempo: %lf  no: %d",valMelhorSolucao, tGeral.getCPUTotalSecs(),nosVisitados); 
    tGeral.start();
    fflush(logSaida);*/
    
    *pct_tree_solved += pow(0.5, static_cast<double>(depth));
    #ifdef MENSAGEM_BB 
      printf("Total da Arvore Resolvida: %lf\n--------------------------------\n",acumulador);
    #endif

    // Se temos todas as variáveis inteiras e já resolvemos toda
    // a árvore, temos a solução ótima.
    if(*pct_tree_solved > 1.0 - Globals::EPS()) {
      *status = OPTSTAT_MIPOPTIMAL;
    }

    return new_lower_bound;
  }

  #ifdef MENSAGEM_BB 
    printf("Variaveis Inteiras: %d\n-----------------------------------\n",contInteiro);
  #endif

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
                           depth + 1,  // TODO(danielrocha): existe mesmo esse '+1'??
                           pct_tree_solved,
                           num_visited_nodes,
                           status);  
  
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
                           depth + 1,  // TODO(danielrocha): existe mesmo esse '+1'??
                           pct_tree_solved,
                           num_visited_nodes,
                           status); 

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
      // TODO(danielrocha): log error
      assert(false);
  }
}

bool SolverGeracaoColunas::ShouldRemoveColumnWhenFixing(
    FixingSense sense, const VariableGeracaoColunas& var,
    int fixed_machine, int fixed_task) {
  switch (sense) {
    case FIX_ON_ZERO:
      // Se a coluna 'i' se refere à máquina 'fixed_machine'
      if (fixed_machine == var.column().machine()) {
        // TODO(danielrocha):
        // CHECK(lp->getCoef(p.num_tasks() + fixed_machine, i) > 1.0 - Globals::EPS()
        const vector<int>& tasks = var.column().tasks();
        // Se nessa coluna 'fixed_task' esta associada à 'fixed_machine'
        if (find(tasks.begin(), tasks.end(), fixed_task) != tasks.end()) {
          // TODO(danielrocha): add CHECKS
          assert(lp_->getCoef(fixed_task, fixed_machine) > 1.0 - Globals::EPS());
          //assert(lp_->getObj(index) == var.cost());
          return true;
        }
      }
      return false;

    case FIX_ON_ONE: {
      const vector<int>& tasks = var.column().tasks();
      bool machine_on_column = fixed_machine == var.column().machine();
      bool task_on_column =
        find(tasks.begin(), tasks.end(), fixed_task) != tasks.end();
      if ((machine_on_column && !task_on_column) ||
          (!machine_on_column && task_on_column)) {
        // TODO(danielrocha): add checks
        //assert((lp_->getCoef(p.num_tasks() + fixed_machine, i) < Globals::EPS() &&
        //        lp_->getCoef(fixed_task, i) > 1 - Globals::EPS()) ||
        //       (lp_->getCoef(p.num_tasks() + fixed_machine, i) > 1.0 - Globals::EPS() &&
        //        lp_->getCoef(fixed_task, i) < Globals::EPS()));
        return true;
      }
      return false;
    }

    default:
      // TODO(danielrocha): log(error)
      assert(false);
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

  // Calcula quantas colunas serão removidas
  vector<VariableGeracaoColunas> removed_vars;
  vector<int> removed_indices;
  for (VariableGeracaoColunasHash::iterator it = vHash_.begin();
       it != vHash_.end(); /* incremented at the end */) {
    bool remove_this_var = false;
    const VariableGeracaoColunas& var = it->first;
    int index = it->second;
    if (var.type() == VariableGeracaoColunas::COL &&
        ShouldRemoveColumnWhenFixing(fixing_sense, var,
                                     fixed_machine, fixed_task)) {
      removed_indices.push_back(index);
      removed_vars.push_back(var);
      remove_this_var = true;
    }

    if (remove_this_var) {
      VariableGeracaoColunasHash::iterator remove = it;
      ++it;
      vHash_.erase(remove);
    } else {
      ++it;
    }
  }

  lp->delSetCols(removed_indices.size(), &removed_indices[0]);

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
    var->set_column_index(column_count_++);
    AddNewColumn(*var, 0.0, OPT_INF, &vHash_, lp);
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
    lp->optimize(METHOD_DUAL);

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
  }

  return num_integer_vars;
}

/********************************************************************
**             VARIABLE CREATION + STABILIZATION                   **
*********************************************************************/

void SolverGeracaoColunas::AddStabilizationColumnsWithCoeficients(
    const ProblemData& p, const vector<double>& relaxed_dual_values,
    VariableGeracaoColunasHash* vHash, OPT_LP* lp) {
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
    (*vHash)[var] = lp->getNumCols();
    double lower_bound = 0.0, upper_bound = 0.0;
    lp->newCol(
      OPT_COL(OPT_COL::VAR_CONTINUOUS, var.cost(), lower_bound, upper_bound),
      var.ToString().c_str());

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
    (*vHash)[var] = lp->getNumCols();
    double lower_bound = 0.0, upper_bound = 0.0;
    lp->newCol(
      OPT_COL(OPT_COL::VAR_CONTINUOUS, var.cost(), lower_bound, upper_bound),
      var.ToString().c_str());

    // Adjusts the matrix coeficient
    // TODO(danielrocha): maybe look for the constraint in the hash
    lp->chgCoef(index, lp->getNumCols() - 1, -1.0);
  }
}

void SolverGeracaoColunas::AddIntegerSolutionColumnsWithCoeficients(
    const ProblemData& p, const ProblemSolution& int_sol,
    VariableGeracaoColunasHash* vHash, OPT_LP* lp) {
  //colocando solucao inteira
  for(int mac = 0; mac < p.num_machines(); ++mac) {  
    vector<int> tasks_assigned;
    int_sol.GetMachineAssignments(mac, &tasks_assigned);
    AddNewColumn(int_sol.AssignmentCost(mac), 0.0, OPT_INF,
                 mac, tasks_assigned, &vHash_, lp);
  }
}

// TODO(danielrocha): remover o parametro 'index'
void SolverGeracaoColunas::AddNewColumn(double cost, double lower_bound,
                                        double upper_bound, int machine,
                                        const vector<int>& tasks,
                                        VariableGeracaoColunasHash* vHash,
                                        OPT_LP* lp) {
  Column column(machine, column_count_, tasks);
  VariableGeracaoColunas var;
  var.set_type(VariableGeracaoColunas::COL);
  var.set_machine(machine);
  var.set_column(column);
  var.set_cost(cost);
  // TODO(danielrocha): check
  assert(vHash->find(var) == vHash->end());

  int column_index = lp->getNumCols();
  (*vHash)[var] = column_index;
  lp->newCol(OPT_COL(OPT_COL::VAR_CONTINUOUS, var.cost(), lower_bound, upper_bound),
             var.ToString().c_str());

  // TODO(danielrocha): maybe look for the constraints in the hash
  lp->chgCoef(problem_data_->num_tasks() + machine, column_index, 1.0);
  for(int task_idx = 0; task_idx < static_cast<int>(tasks.size()); ++task_idx)
    lp->chgCoef(tasks[task_idx], column_index, 1.0);

  column_count_++;
}

void SolverGeracaoColunas::AddNewColumn(const VariableGeracaoColunas& var,
                                        double lower_bound, double upper_bound, 
                                        VariableGeracaoColunasHash* vHash,
                                        OPT_LP* lp) {
  // TODO(danielrocha): check
  assert(vHash->find(var) == vHash->end());

  int column_index = lp->getNumCols();
  (*vHash)[var] = column_index;
  lp->newCol(OPT_COL(OPT_COL::VAR_CONTINUOUS, var.cost(), lower_bound, upper_bound),
             var.ToString().c_str());

  // TODO(danielrocha): maybe look for the constraints in the hash
  lp->chgCoef(problem_data_->num_tasks() + var.column().machine(), column_index, 1.0);
  for (int task_idx = 0;
       task_idx < static_cast<int>(var.column().tasks().size()); ++task_idx)
    lp->chgCoef(var.column().tasks()[task_idx], column_index, 1.0);
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
      // TODO(danielrocha): add a log(error) here, this should never happen
      assert(false);
      continue;
    }

    (*cHash)[cons] = lp->getNumRows();
    lp->addRow(OPT_ROW(OPT_ROW::EQUAL, 1.0), cons.ToString().c_str());
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
      // TODO(danielrocha): add a log(error) here, this should never happen
      assert(false);
      continue;
    }

    (*cHash)[cons] = lp->getNumRows();
    lp->addRow(OPT_ROW(OPT_ROW::LESS, 1.0), cons.ToString().c_str());
  }
}

/********************************************************************
**                SOLVING AND RESULTS METHODS                      **
*********************************************************************/

OPTSTAT SolverGeracaoColunas::SolveWithCutoff(int time_limit, double cut_off, bool first) {
  // Acha os valores das variáveis duais na formulação relaxada
  vector<double> dual_values;
  SolverFormulacaoPadrao::GetRelaxedDualValues(*problem_data_, &dual_values);

  // Pega uma solução inteira qualquer
  ProblemSolution integer_solution(problem_data_);
  SolverFormulacaoPadrao::GetFirstIntegerSolution(*problem_data_,
                                                  &integer_solution);
  if (first) {
    return OPTSTAT_FEASIBLE;
  }

  // O resolvedor a ser utilizado: é importante notar que o resolvedor é LP, ou
  // seja, vamos fazer B&B para buscar a solução ótima.
  OPT_LP* lp = new OPT_CPLEX;
  lp_ = lp;
  lp->createLP("FormulacaoGeracaoColunas", OPTSENSE_MINIMIZE, PROB_LP);
  SetUpCplexParams(lp);

  AddSumAssignmentsPerTaskEqualsOneConstraints(*problem_data_, &cHash_, lp);
  AddSumAssignmentsPerMachineAtMostOneConstraints(*problem_data_, &cHash_, lp);

  // Adiciona as colunas para a estabilizacao da geracao de colunas
  AddStabilizationColumnsWithCoeficients(*problem_data_, dual_values, &vHash_, lp);
  // Adiciona a solução inteira ao modelo
  AddIntegerSolutionColumnsWithCoeficients(*problem_data_, integer_solution, &vHash_, lp);

  lp->optimize(METHOD_PRIMAL);

  if (cut_off > 0.0 && cut_off < integer_solution.cost())
    integer_solution.set_cost(static_cast<int>(cut_off + Globals::EPS()));

  // Colocar aqui um cronometro de tempo!
  //tGeral.stop();
  //tGeral.reset();
  //tGeral.start();

  int num_nodes_limit = 1000000;
  OPTSTAT status =
    SetUpAndRunBranchAndBound(*problem_data_,
                              num_nodes_limit,
                              lp,
                              &integer_solution);  

  //tGeral.stop();

  // Nesse ponto, devemos ser capazes de chamar GenerateSolution()
  // Post-Condition: GenerateSolution() == integer_solution
  // TODO(danielrocha)
  return status;
}

void SolverGeracaoColunas::GenerateSolution(const ProblemData& p,
                                            const VariableGeracaoColunasHash& vHash,
                                            const OPT_LP& lp,
                                            ProblemSolution* sol) const {
  // Loads the solution into the Variable hash and constructs the ProblemSolution
  vector<double> current_sol(lp.getNumCols());
  lp.getX(&current_sol[0]);

  vector<vector<double> > x(p.num_machines(),
                            vector<double>(p.num_tasks(), 0.0));
  GetXijMatrix(current_sol, vHash, &x);

  sol->Clear();
  for (int machine = 0; machine < p.num_machines(); ++machine) {
    for (int task = 0; task < p.num_tasks(); ++task) {
      if (x[machine][task] > Globals::EPS())
        sol->set_assignment(task, machine);
    }
  }

  // TODO(danielrocha): check
  assert(sol->IsValid());
}


void SolverGeracaoColunas::SolveWithCutoff(int time_limit, double cut_off,
                                           bool first, SolverStatus* status) {
	status->status = SolveWithCutoff(time_limit, cut_off, first);
	status->gap_absolute = GetGapAbsolute();
	status->gap_relative = GetGapRelative();
	if (status->status == OPTSTAT_FEASIBLE || status->status == OPTSTAT_MIPOPTIMAL)
		GenerateSolution(*problem_data_, vHash_, *lp_, &status->final_sol);
}

void SolverGeracaoColunas::Solve(int time_limit, SolverStatus* status) {
	SolveWithCutoff(time_limit, Globals::Infinity(), false, status);
}

int SolverGeracaoColunas::Solve() {
  SolverStatus status;
  Solve(60 * 60, &status);
  return 0;
}

double SolverGeracaoColunas::GetGapRelative() {
	return lp_->getMIPRelGap();
}

double SolverGeracaoColunas::GetGapAbsolute() {
	return lp_->getMIPAbsGap();
}
