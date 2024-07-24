//
// Created by crl on 22/07/2024.
//

#ifndef BACKWARD_SEARCH_H
#define BACKWARD_SEARCH_H

#include <algorithm>
#include <ctime>

#include "data_structures/adjacency_matrix.h"
#include "data_structures/apex_path_pair.h"
#include "data_structures/map_queue.h"
#include "data_structures/node.h"
#include "definitions.h"
#include "solvers/apex.h"

#include <unordered_map>

using BackwardSearchSolutionSet = std::unordered_map<size_t, ApexSolutionSet>;

class BackwardSearch {
public:
  const AdjacencyMatrix &adj_matrix;
  EPS eps;

  std::clock_t start_time = std::clock();
  size_t num_expansion = 0;
  size_t num_generation = 0;

  std::vector<float> min_g2;
  std::vector<std::vector<ApexPathPairPtr>> expanded;

  ApexPathPairPtr last_solution = nullptr;

  MultiValuedHeuristic operator()(const size_t &source, const size_t &target,
                                  const Heuristic &heuristic_to_target,
                                  const Heuristic &heuristic_to_source);
  virtual ~BackwardSearch() = default;

  virtual void insert(ApexPathPairPtr &ap, MapQueue &queue);
  bool is_dominated(const ApexPathPairPtr &ap) const;
  bool local_dominance_check(const ApexPathPairPtr &ap) const {
    return (ap->apex->g[1] >= min_g2[ap->id]);
  }
  bool global_dominance_check(const ApexPathPairPtr &ap) const;
  void merge_to_apex_list(const ApexPathPairPtr &ap,
                          ApexSolutionSet &solutions) const;
  BackwardSearch(const AdjacencyMatrix &adj_matrix, EPS eps);
  void set_min_g2(const ApexPathPairPtr &ap) {
    min_g2[ap->id] = ap->apex->g[1];
  }
};

#endif // BACKWARD_SEARCH_H
