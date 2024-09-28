//
// Created by crl on 15/07/2024.
//

#include "solvers/apex.h"

#include <iostream>

ApexSearch::ApexSearch(const AdjacencyMatrix &adj_matrix, const EPS &eps)
    : AbstractSolver(adj_matrix, eps) {
  expanded.resize(adj_matrix.size() + 1);
  min_g2.resize(adj_matrix.size() + 1, static_cast<float>(MAX_COST));
}

void ApexSearch::insert(ApexPathPairPtr &ap, MapQueue &queue) {
  std::list<ApexPathPairPtr> &relevant_aps = queue.get_open(ap->id);
  for (auto relevant_ap = relevant_aps.begin();
       relevant_ap != relevant_aps.end(); ++relevant_ap) {
    if ((*relevant_ap)->is_active == false) {
      relevant_aps.erase(relevant_ap);
      continue;
    }
    if (ap->update_nodes_by_merge_if_bounded(*relevant_ap, eps)) {
      if ((ap->apex != (*relevant_ap)->apex) ||
          (ap->path_node != (*relevant_ap)->path_node)) {
        (*relevant_ap)->is_active = false;
        relevant_aps.erase(relevant_ap);
        queue.insert(ap);
      }
      return;
    }
  }
  queue.insert(ap);
}

void ApexSearch::merge_to_solutions(const ApexPathPairPtr &ap,
                                    ApexSolutionSet &solutions) {
  for (auto &solution : solutions) {
    if (solution->update_nodes_by_merge_if_bounded(ap, eps)) {
      return;
    }
  }
  solutions.push_back(ap);
  last_solution = ap;
}

bool ApexSearch::global_dominance_check(const ApexPathPairPtr &ap) const {
  if (last_solution == nullptr) {
    return false;
  }
  if (is_bounded(ap->apex, last_solution->path_node, eps)) {
    last_solution->update_apex_by_merge_if_bounded(ap->apex, eps);
    return true;
  }
  return false;
}

bool ApexSearch::is_dominated(const ApexPathPairPtr &ap) const {
  if (local_dominance_check(ap)) {
    return true;
  }
  return global_dominance_check(ap);
}

void ApexSearch::init_search() {
  AbstractSolver::init_search();
  expanded.clear();
  expanded.resize(adj_matrix.size() + 1);
}
void ApexSearch::operator()(const size_t &source, const size_t &target,
                            const Heuristic &heuristic, SolutionSet &solutions,
                            unsigned int time_limit) {

  init_search();
  start_time = std::clock();

  ApexSolutionSet ap_solutions;

  MapQueue open(adj_matrix.size() + 1);

  NodePtr source_node = std::make_shared<Node>(source, std::vector<float>(2, 0),
                                               heuristic(source));
  ApexPathPairPtr ap =
      std::make_shared<ApexPathPair>(source_node, source_node, heuristic(source));
  open.insert(ap);

  while (!open.empty()) {
    if ((std::clock() - start_time) / CLOCKS_PER_SEC > time_limit) {
      for (auto &ap_solution : ap_solutions) {
        solutions.push_back(ap_solution->path_node);
      }
      return;
    }
    ap = open.pop();
    num_generation += 1;

    if (ap->is_active == false) {
      continue;
    }

    if (is_dominated(ap)) {
      continue;
    }

    set_min_g2(ap);

    num_expansion += 1;

    expanded[ap->id].push_back(ap);

    if (ap->id == target) {
      merge_to_solutions(ap, ap_solutions);
      continue;
    }

    const std::vector<Edge> &outgoing_edges = adj_matrix[ap->id];
    for (const auto &outgoing_edge : outgoing_edges) {
      ApexPathPairPtr next_ap =
          std::make_shared<ApexPathPair>(ap, outgoing_edge, heuristic(outgoing_edge.target));

      if (is_dominated(next_ap)) {
        continue;
      }

      this->insert(next_ap, open);
    }
  }

  for (auto &ap_solution : ap_solutions) {
    solutions.push_back(ap_solution->path_node);
  }
}
