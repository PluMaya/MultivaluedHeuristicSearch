//
// Created by crl on 22/07/2024.
//

#include <assert.h>
#include <multivalued_heuristic/backward_search.h>

#include <boost/mpl/assert.hpp>
#include <utility>

void BackwardSearch::insert(ApexPathPairPtr &ap, MapQueue &queue) {
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

bool BackwardSearch::global_dominance_check(const ApexPathPairPtr &ap) const {
  if (last_solution == nullptr) {
    return false;
  }
  if (is_bounded(ap->apex, last_solution->path_node, eps)) {
    last_solution->update_apex_by_merge_if_bounded(ap->apex, eps);
    return true;
  }
  return false;
}
BackwardSearch::BackwardSearch(const AdjacencyMatrix &adj_matrix, EPS eps)
    : adj_matrix(adj_matrix), eps(std::move(eps)) {
  expanded.resize(adj_matrix.size() + 1);
  min_g2.resize(adj_matrix.size() + 1, static_cast<float>(MAX_COST));
}
void BackwardSearch::merge_to_apex_list(const ApexPathPairPtr &ap,
                                        ApexSolutionSet &solutions) const {
  for (auto &solution : solutions) {
    if (solution->update_nodes_by_merge_if_bounded(ap, eps)) {
      return;
    }
  }
  solutions.push_back(ap);
}

bool BackwardSearch::is_dominated(const ApexPathPairPtr &ap) const {
  if (local_dominance_check(ap)) {
    return true;
  }
  return global_dominance_check(ap);
}

std::vector<std::vector<float>>
make_list_of_valeus(const ApexSolutionSet &apex_solution_set,
                    const std::vector<float> &heuristic_value) {
  std::vector<std::vector<float>> result = {};
  if (apex_solution_set.empty()) {
    result.push_back(heuristic_value);
  } else {
    for (auto &solution : apex_solution_set) {
      result.push_back(solution->apex->f);
    }
    result.back()[1] = heuristic_value[1];
  }
  return result;
}

MultiValuedHeuristic
BackwardSearch::operator()(const size_t &source, const size_t &target,
                           const Heuristic &heuristic_to_target,
                           const Heuristic &heuristic_to_source) {

  start_time = std::clock();

  BackwardSearchSolutionSet backward_search_solutions;

  MapQueue open(adj_matrix.size() + 1);

  NodePtr source_node = std::make_shared<Node>(source, std::vector<float>(2, 0),
                                               heuristic_to_target(source));
  ApexPathPairPtr ap = std::make_shared<ApexPathPair>(source_node, source_node,
                                                      heuristic_to_target);
  open.insert(ap);

  while (!open.empty()) {
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
    merge_to_apex_list(ap, backward_search_solutions[ap->id]);

    if (ap->id == target) {
      last_solution = ap;
      continue;
    }

    const std::vector<Edge> &outgoing_edges = adj_matrix[ap->id];
    for (const auto &outgoing_edge : outgoing_edges) {
      ApexPathPairPtr next_ap =
          std::make_shared<ApexPathPair>(ap, outgoing_edge);

      if (is_dominated(next_ap)) {
        continue;
      }

      this->insert(next_ap, open);
    }
  }

  std::unordered_map<size_t, std::vector<std::vector<float>>> mvh_results;
  for (size_t i = 0; i < adj_matrix.size(); ++i) {
    mvh_results.emplace(i, make_list_of_valeus(backward_search_solutions[i],
                                               heuristic_to_source(i)));
  }

  return [mvh_results](size_t vertex) -> std::vector<std::vector<float>> {
    return mvh_results.find(vertex)->second;
  };
}
