//
// Created by crl on 27/07/2024.
//

#include "multivalued_heuristic/forward_search.h"

#include "solvers/apex.h"

#include <optional>

ForwardSearch::ForwardSearch(const AdjacencyMatrix &adj_matrix, const EPS &eps)
    : AbstractSolver(adj_matrix, eps) {
  min_g2.resize(adj_matrix.size() + 1,
                {static_cast<float>(MAX_COST), static_cast<float>(MAX_COST)});
}

bool ForwardSearch::local_dominance_check(const NodePtr &node_ptr) const {
  if (min_g2[node_ptr->id][1] >= node_ptr->g[1]) {
    return false;
  }
  if (min_g2[node_ptr->id][0] > node_ptr->g[0]) {
    std::cout <<min_g2[node_ptr->id][0] << ',' << node_ptr->g[0] <<std::endl;
    return false; // todo: see later if to change
  }
  return true;
}

bool ForwardSearch::global_dominance_check(const NodePtr &node_ptr,
                                           const size_t &target) const {
  return (node_ptr->f[1] > min_g2[target][1]);
}

bool comp(const std::vector<float> &a, const std::vector<float> &b) {
  return a[1] > b[1];
}

std::optional<std::vector<float>> get_first_undominated_heuristic_value(
    const std::vector<std::vector<float>> &pairs_list,
    const std::vector<float> &g_value, const float target_g2) {
  // Use upper_bound with the custom comparator to find the position
  if (target_g2 <= g_value[1]) {
    return std::nullopt;
  }
  const std::vector value_to_compare = {g_value[0], target_g2 - g_value[1]};
  const auto upper = std::upper_bound(pairs_list.begin(), pairs_list.end(),
                                      value_to_compare, comp);
  // we need the first element that h[1] + g[1] will be less than target g[1].
  // which means, we need the element "right" to the desired location to insert
  // g-value, assuming we would have wanted to insert it by it's second vector
  // entry.
  if (upper == pairs_list.end()) {
    // If target is less than all elements, return None to indicate that there
    // is no relevant value for the heuristic
    return std::nullopt;
  }
  // else, return the value derived from the upper bound
  return std::optional{(*upper)};
}

bool min_based_comp(const std::vector<float> &a, const std::vector<float> &b) {
  return a[1] < b[1];
};

void ForwardSearch::operator()(const size_t &source, const size_t &target,
                               const MultiValuedHeuristic &heuristic,
                               SolutionSet &solutions) {
  init_search();
  start_time = std::clock();

  std::priority_queue<NodePtr, std::vector<NodePtr>, CompareNodeByFValue> open;
  std::vector<NodePtr> close;

  auto source_heuristic_value = heuristic(source)[0];
  NodePtr source_node = std::make_shared<Node>(source, std::vector<float>(2, 0),
                                               source_heuristic_value);
  open.push(source_node);

  int i = 0;
  while (!open.empty()) {
    i +=1;
    if (i % 1000 == 0) {
      std::cout << open.size() << std::endl;
    }
    auto node = open.top();
    open.pop();
    num_generation += 1;

    if (local_dominance_check(node)) {
      continue;
    }

    if (global_dominance_check(node, target)) {
      if (auto new_heuristic_value = get_first_undominated_heuristic_value(
              heuristic(node->id), node->g, min_g2[target][1])) {
        NodePtr new_node = std::make_shared<Node>(node->id, node->g,
                                                  new_heuristic_value.value(), node);
        // todo: insert to sorted list
        open.push(new_node);
      }
      continue;
    }

    min_g2[node->id] = std::min(min_g2[node->id], node->g, min_based_comp);
    num_expansion += 1;
    close.push_back(node);
    // todo: see later if to add into a sorted pareto list

    if (node->id == target) {
      std::cout << "target was found" << std::endl;
      solutions.push_back(node);
      continue;
    }

    const std::vector<Edge> &outgoing_edges = adj_matrix[node->id];
    for (const auto &outgoing_edge : outgoing_edges) {
      std::vector<float> new_g(node->g);
      for (int i = 0; i < new_g.size(); i++) {
        new_g[i] += outgoing_edge.cost[i];
      }
      auto new_h = get_first_undominated_heuristic_value(heuristic(outgoing_edge.target), new_g, min_g2[target][1]);
      if (new_h == std::nullopt) {
        continue;
      }
      NodePtr successor_node =
          std::make_shared<Node>(outgoing_edge.target, new_g, new_h.value(), node);

      if (local_dominance_check(successor_node)) {
        continue;
      }

      open.push(successor_node);
    }
  }
}