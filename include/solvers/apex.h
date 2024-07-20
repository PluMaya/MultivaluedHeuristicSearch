//
// Created by crl on 15/07/2024.
//

#ifndef APEX_H
#define APEX_H
#include "abstract_solver.h"
#include "data_structures/apex_path_pair.h"
#include "data_structures/map_queue.h"

using ApexSolutionSet = std::vector<ApexPathPairPtr>;

class ApexSearch : public AbstractSolver {
public:
  std::vector<float> min_g2;
  std::vector<std::vector<ApexPathPairPtr>> expanded;
  ApexPathPairPtr last_solution = nullptr;

  std::string get_solver_name() override { return "A*pex"; }

  void operator()(const size_t &source, const size_t &target, const Heuristic &heuristic,
                  SolutionSet &solutions, unsigned int time_limit) override;

  virtual void insert(ApexPathPairPtr &ap, MapQueue &queue);
  bool is_dominated(const ApexPathPairPtr &ap) const;
  bool local_dominance_check(const ApexPathPairPtr &ap) const {
    return (ap->apex->g[1] >= min_g2[ap->id]);
  }
  bool global_dominance_check(const ApexPathPairPtr &ap) const;
  void merge_to_solutions(const ApexPathPairPtr &ap,
                          ApexSolutionSet &solutions);
  ApexSearch(const AdjacencyMatrix &adj_matrix, const EPS &eps);
  void set_min_g2(const ApexPathPairPtr &ap) {
    min_g2[ap->id] = ap->apex->g[1];
  }
  void init_search() override;
};

#endif // APEX_H
