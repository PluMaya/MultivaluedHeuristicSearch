//
// Created by crl on 27/07/2024.
//

#ifndef FORWARD_SEARCH_H
#define FORWARD_SEARCH_H
#include "data_structures/adjacency_matrix.h"
#include "data_structures/apex_path_pair.h"
#include "definitions.h"
#include "solvers/abstract_solver.h"
#include <iostream>
#include <optional>
#include <set>

struct CustomComparator {
  bool operator()(const NodePtr &a, const NodePtr &b) const {
    return (a->g[0] < b->g[0] || ((a->g[0] == b->g[0]) && a->g[1] < b->g[1]));
  };
};

// Type alias for the set with the custom comparator
using CustomSet = std::set<NodePtr, CustomComparator>;

class ForwardSearch : public AbstractSolver {
public:
  static bool min_based_comp(const std::vector<float> &a,
                             const std::vector<float> &b);
  size_t inconsistencis = 0;
  size_t local_dominance_checks_invocations = 0;
  size_t true_local_dominance_chekcs = 0;
  size_t global_dominance_checks_invocations = 0;
  size_t true_global_dominance_checks = 0;
  std::vector<std::vector<float>> min_g2;
  std::vector<CustomSet> pareto_list;

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

  [[nodiscard]] bool local_dominance_check(const NodePtr &node_ptr);

  [[nodiscard]] bool local_dominance_check_no_dr(const NodePtr &node_ptr);

  [[nodiscard]] static bool global_dominance_check(const NodePtr &node_ptr,
                                            const SolutionSet &solutions);

  static bool comp(const std::vector<float> &a, const std::vector<float> &b);

  std::optional<std::vector<float>> get_first_undominated_heuristic_value(
      const std::vector<float> &g_value, const size_t &target,
      const std::vector<std::vector<float>> &node_mvh);

  std::optional<std::vector<float>> get_first_undominated_heuristic_value_naive(
      const std::vector<float> &g_value, const size_t &target,
      const std::vector<std::vector<float>> &node_mvh);
};

#endif // FORWARD_SEARCH_H
