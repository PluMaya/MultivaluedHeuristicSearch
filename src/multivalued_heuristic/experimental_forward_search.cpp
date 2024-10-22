//
// Created by crl on 27/07/2024.
//

#include "multivalued_heuristic/experimental_forward_search.h"

#include "solvers/apex.h"

#include <cassert>
#include <optional>
#include <chrono>

/**
 * initialize the data structures in FW algorithm
 *
 * @param adj_matrix adjacency matrix of the graph
 * @param eps epsilon (not really required)
 */
ExperimentalForwardSearch::ExperimentalForwardSearch(
    const AdjacencyMatrix &adj_matrix, const EPS &eps)
    : AbstractSolver(adj_matrix, eps) {
  // At the beginning, min-g2 values are all set to max value
  min_g2.resize(adj_matrix.size() + 1,
                {static_cast<float>(MAX_COST), static_cast<float>(MAX_COST)});
  pareto_list.resize(adj_matrix.size() + 1, CustomSet());
}

bool ExperimentalForwardSearch::naive_local_dominance_check(
    const NodePtr &node_ptr) {
  auto pareto_list_id = pareto_list[node_ptr->id];
  for (auto &pareto_list_id_element : pareto_list_id) {
    if (pareto_list_id_element->g[0] <= node_ptr->g[0] &&
        pareto_list_id_element->g[1] <= node_ptr->g[1]) {
      return true;
    }
  }
  return false;
}

bool ExperimentalForwardSearch::local_dominance_check(const NodePtr &node_ptr) {
  std::chrono::time_point<std::chrono::system_clock> start = std::chrono::high_resolution_clock::now();
  // in the trivial case, the node is pruned
  if (min_g2[node_ptr->id][1] > node_ptr->g[1]) { // g2 smaller
    time_map["local_dominance_check"] += std::chrono::high_resolution_clock::now() - start;
    return false;
  }
  // if the node in g2-min has a larger g[0] value than the current node,
  // we need to be careful and perform a full dominance check (without dr)
  if (min_g2[node_ptr->id][0] > node_ptr->g[0]) { // g2 larger equal, g1 smaller
    auto pareto_list_id = pareto_list[node_ptr->id];
    const auto it = pareto_list_id.lower_bound(node_ptr);
    if ((*it)->g == node_ptr->g) { // if we encountered a node with the same
      // value, means it is dominated by it
      time_map["local_dominance_check"] += std::chrono::high_resolution_clock::now() - start;
      return true;
    }
    if (it == pareto_list_id.begin()) { // if the iterator is the begininng,
      // means there is no dominating solution
      time_map["local_dominance_check"] += std::chrono::high_resolution_clock::now() - start;
      return false;
    }
    auto prev_it = std::prev(it);
    time_map["local_dominance_check"] += std::chrono::high_resolution_clock::now() - start;
    return ((*prev_it)->g[1] <=
            node_ptr->g[1]); // if the g2-value of prev is smaller, means it is
    // dominateing the current value
  }
  // else, we can assume dominance check is true using DR
  time_map["local_dominance_check"] += std::chrono::high_resolution_clock::now() - start;
  return true;
}

bool ExperimentalForwardSearch::naive_global_dominance_check(
    const NodePtr &node_ptr, const SolutionSet &solutions) {
  auto start = std::chrono::high_resolution_clock::now();
  for (auto &sol : solutions) {
    if (sol->g[0] <= node_ptr->f[0] && sol->g[1] <= node_ptr->f[1]) {
      time_map["global_dominance_check"] += std::chrono::high_resolution_clock::now() - start;
      return true;
    }
  }
  time_map["global_dominance_check"] += std::chrono::high_resolution_clock::now() - start;
  return false;
}

bool ExperimentalForwardSearch::global_dominance_check(
    const NodePtr &node_ptr, const size_t &target_id) {
  auto start = std::chrono::high_resolution_clock::now();

  auto pareto_list_id = pareto_list[target_id];
  const auto it = pareto_list_id.lower_bound(node_ptr);
  if (it == pareto_list_id.begin()) { // no dominating vlaue
    time_map["global_dominance_check"] += std::chrono::high_resolution_clock::now() - start;
    return false;
  }
  if (it == pareto_list_id.end()) {
    auto prev_it = std::prev(it);
    time_map["global_dominance_check"] += std::chrono::high_resolution_clock::now() - start;
    return ((*prev_it)->g[1] <= node_ptr->f[1]);
  }
  if ((*it)->g == node_ptr->g) { // there exists a solution with the same
    time_map["global_dominance_check"] += std::chrono::high_resolution_clock::now() - start;
    return true;
  }

  // else, perform dominance check with DR
  auto prev_it = std::prev(it);
  time_map["global_dominance_check"] += std::chrono::high_resolution_clock::now() - start;
  return ((*prev_it)->g[1] <= node_ptr->f[1]);
}

bool ExperimentalForwardSearch::comp(const std::vector<float> &a,
                                     const std::vector<float> &b) {
  return a[1] > b[1] || (a[1] == b[1] && a[0] > b[0]);
}

std::optional<std::vector<float>>
ExperimentalForwardSearch::naive_get_first_undominated_heuristic_value(
    const std::vector<float> &g_value, const size_t &target,
    const std::vector<std::vector<float>> &node_mvh) {
  auto start = std::chrono::high_resolution_clock::now();
  for (auto &heuristic : node_mvh) {
    std::vector new_value = {heuristic[0] + g_value[0],
                             heuristic[1] + g_value[1]};
    if (new_value[1] < min_g2[target][1]) {
      time_map["get_first_undominated_heuristic_value"] += std::chrono::high_resolution_clock::now() - start;
      return std::optional{heuristic};
    }
  }
  time_map["get_first_undominated_heuristic_value"] += std::chrono::high_resolution_clock::now() - start;
  return std::nullopt;
}

std::optional<std::vector<float>>
ExperimentalForwardSearch::get_first_undominated_heuristic_value(
    const std::vector<float> &g_value, const size_t &target,
    const std::vector<std::vector<float>> &node_mvh) {
  auto start = std::chrono::high_resolution_clock::now();
  // Use upper_bound with the custom comparator to find the position
  if (min_g2[target][1] <= g_value[1]) {
    time_map["get_first_undominated_heuristic_value"] += std::chrono::high_resolution_clock::now() - start;
    auto cur = std::chrono::high_resolution_clock::now();
    return std::nullopt;
  }
  const std::vector v = {static_cast<float>(MAX_COST),
                         static_cast<float>(MAX_COST)};
  if (min_g2[target] == v) { // if target was not encountered, return the first
    // heuristic value in the mvh list
    time_map["get_first_undominated_heuristic_value"] += std::chrono::high_resolution_clock::now() - start;
    return std::optional{node_mvh[0]};
  }

  const std::vector value_to_compare = {min_g2[target][0] - g_value[0],
                                        min_g2[target][1] - g_value[1]};
  const auto upper = std::upper_bound(node_mvh.begin(), node_mvh.end(),
                                      value_to_compare, comp);
  // we need the first element that h[1] + g[1] will be less than target g[1].
  // which means, we need the element "right" to the desired location to insert
  // g-value, assuming we would have wanted to insert it by it's second vector
  // entry.

  if (upper == node_mvh.end()) {
    // If target is less than all elements, return None to indicate that there
    // is no relevant value for the heuristic
    time_map["get_first_undominated_heuristic_value"] += std::chrono::high_resolution_clock::now() - start;
    return std::nullopt;
  }
  time_map["get_first_undominated_heuristic_value"] += std::chrono::high_resolution_clock::now() - start;
  // else, return the value derived from the upper bound
  return std::optional{(*upper)};
}

bool ExperimentalForwardSearch::min_based_comp(const std::vector<float> &a,
                                               const std::vector<float> &b) {
  return (a[1] < b[1] || ((a[1] == b[1]) && a[0] < b[0]));
};

void ExperimentalForwardSearch::operator()(
    const size_t &source, const size_t &target,
    const MultiValuedHeuristic &heuristic, SolutionSet &solutions) {
  init_search();
  start_time = std::clock();
  auto other_start_time = std::chrono::high_resolution_clock::now();
  auto init = std::chrono::high_resolution_clock::now();
  time_map["local_dominance_check"] = init - init;
  time_map["global_dominance_check"] = init - init;
  time_map["get_first_undominated_heuristic_value"] = init - init;
  time_map["heuristic_rerieval"] = init - init;

  std::priority_queue<NodePtr, std::vector<NodePtr>, CompareNodeByFValue> open;

  auto start = std::chrono::high_resolution_clock::now();
  auto cur_heuristic = heuristic(source);
  time_map["heuristic_rerieval"] += std::chrono::high_resolution_clock::now() - start;

  auto source_heuristic_value = cur_heuristic[0];
  NodePtr source_node = std::make_shared<Node>(source, std::vector<float>(2, 0),
                                               source_heuristic_value);
  open.push(source_node);

  while (!open.empty()) {
    auto node = open.top();
    open.pop();

    num_generation += 1;

    if (local_dominance_check(node)) {
      continue;
    }

    if (naive_global_dominance_check(node, solutions)) {
      start = std::chrono::high_resolution_clock::now();
      cur_heuristic = heuristic(node->id);
      time_map["heuristic_rerieval"] += std::chrono::high_resolution_clock::now() - start;
      if (auto new_heuristic_value = get_first_undominated_heuristic_value(
              node->g, target, cur_heuristic)) {
        NodePtr new_node = std::make_shared<Node>(
            node->id, node->g, new_heuristic_value.value(), node);
        open.push(new_node);
      }
      continue;
    }

    min_g2[node->id] = std::min(min_g2[node->id], node->g, min_based_comp);
    num_expansion += 1;
    pareto_list[node->id].insert(node);

    if (node->id == target) {
      solutions.push_back(node);
      continue;
    }

    const std::vector<Edge> &outgoing_edges = adj_matrix[node->id];
    for (const auto &outgoing_edge : outgoing_edges) {
      std::vector new_g(node->g);
      for (int i = 0; i < new_g.size(); i++) {
        new_g[i] += outgoing_edge.cost[i];
      }
      start = std::chrono::high_resolution_clock::now();
      cur_heuristic = heuristic(outgoing_edge.target);
      time_map["heuristic_rerieval"] += std::chrono::high_resolution_clock::now() - start;
      auto new_h = get_first_undominated_heuristic_value(
          new_g, target, cur_heuristic);

      if (new_h == std::nullopt) {
        continue;
      }
      NodePtr successor_node = std::make_shared<Node>(
          outgoing_edge.target, new_g, new_h.value(), node);

      if (local_dominance_check(successor_node)) {
        continue;
      }

      open.push(successor_node);
    }
  }
  auto duration = std::chrono::high_resolution_clock::now() - other_start_time;
  runtime = static_cast<float>(std::clock() - start_time);
  for (auto& it: time_map) {
    std::cout << it.first << ' ' << std::chrono::duration_cast<std::chrono::milliseconds>(it.second).count() << "  ";
  }
  std::cout << "runtime " << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() << std::endl;
}