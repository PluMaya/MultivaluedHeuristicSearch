//
// Created by crl on 27/07/2024.
//

#ifndef FORWARD_SEARCH_H
#define FORWARD_SEARCH_H
#include "data_structures/adjacency_matrix.h"
#include "data_structures/apex_path_pair.h"
#include "definitions.h"
#include "solvers/abstract_solver.h"
#include <ctime>
#include <iostream>

class ForwardSearch : public AbstractSolver {
public:
  std::vector<std::vector<float>> min_g2;

  std::string get_solver_name() override { return "ForwardSearch"; }

  void operator()(const size_t &source, const size_t &target,
                  const MultiValuedHeuristic &heuristic,
                  SolutionSet &solutions);

  void operator()(const size_t &source, const size_t &target,
                  const Heuristic &heuristic, SolutionSet &solutions,
                  unsigned time_limit) override {
    std::cout << "not a real func " << std::endl;
  };

  ForwardSearch(const AdjacencyMatrix &adj_matrix, const EPS &eps);

  [[nodiscard]] bool local_dominance_check(const NodePtr &node_ptr) const;

  [[nodiscard]] bool global_dominance_check(const NodePtr &node_ptr,
                              const size_t &target) const;
};

#endif // FORWARD_SEARCH_H
