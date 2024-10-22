//
// Created by crl on 28/09/2024.
//

#ifndef EXPERIMENT_UTILS_H
#define EXPERIMENT_UTILS_H
#include "data_structures/adjacency_matrix.h"
#include "definitions.h"
#include "solvers/abstract_solver.h"

class ExperimentUtils {
public:
  static void run_boa_star(const AdjacencyMatrix &adjecency_matrix,
                           const size_t &source, const size_t &target,
                           const Heuristic &heuristic);

  static void run_apex(const AdjacencyMatrix &adjecency_matrix,
                       const size_t &source, const size_t &target,
                       const EPS &eps, const Heuristic &heuristic);

  static void run_backward_search(const AdjacencyMatrix &adjecency_matrix,
                                  const size_t &source, const size_t &target,
                                  const EPS &eps,
                                  const Heuristic &source_to_target,
                                  const Heuristic &target_to_source);

  static void run_forward_search(const AdjacencyMatrix &adjecency_matrix,
                                 const size_t &source, const size_t &target,
                                 const EPS &eps,
                                 const MultiValuedHeuristic &mvh);

  static void single_run(AdjacencyMatrix &adjecency_matrix, size_t source,
                         size_t target, const std::string &algorithm, const EPS& eps);
};

#endif // EXPERIMENT_UTILS_H
