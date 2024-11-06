//
// Created by crl on 27/07/2024.
//

#include "multivalued_heuristic/forward_search.h"

#include "solvers/apex.h"

#include <cassert>
#include <optional>

/**
 * initialize the data structures in FW algorithm
 *
 * @param adj_matrix adjacency matrix of the graph
 * @param eps epsilon (not really required)
 */
ForwardSearch::ForwardSearch(const AdjacencyMatrix &adj_matrix, const EPS &eps)
    : AbstractSolver(adj_matrix, eps) {
  // At the beginning, min-g2 values are all set to max value
  min_g2.resize(adj_matrix.size() + 1,
                {static_cast<float>(MAX_COST), static_cast<float>(MAX_COST)});
  pareto_list.resize(adj_matrix.size() + 1, CustomSet());
}

/**
 * Perform local dominance check for a given node, using DR when possible
 *
 * @param node_ptr node ptr to perform local-dominance relations
 * @return True if the node is dominated by its g-value, else False
 */
bool ForwardSearch::local_dominance_check(const NodePtr &node_ptr) {
  // in the trivial case, th enode is pruned
  if (min_g2[node_ptr->id][1] > node_ptr->g[1]) { // g2 smaller
    return false;
  }
  // if the node in g2-min has a larger g[0] value than the current node,
  // we need to be careful and perform a full dominance check (without dr)
  if (min_g2[node_ptr->id][0] > node_ptr->g[0]) { // g2 larger equal, g1 smaller
    auto pareto_list_id = pareto_list[node_ptr->id];
    const auto it = pareto_list_id.lower_bound(node_ptr);
    if ((*it)->g == node_ptr->g) { // if we encountered a node with the same
                                   // value, means it is dominated by it
      return true;
    }
    if (it == pareto_list_id.begin()) { // if the iterator is the begininng,
                                        // means there is no dominating solution
      return false;
    }
    auto prev_it = std::prev(it);
    return ((*prev_it)->g[1] <=
            node_ptr->g[1]); // if the g2-value of prev is smaller, means it is
                             // dominateing the current value
  }
  // else, we can assume dominance check is true using DR
  return true;
}

/**
 * Perform local dominance check for a given node, without ever using DR
 *
 * @param node_ptr node ptr to perform local-dominance relations
 * @return True if the node is dominated by its g-value, else False
 */
bool ForwardSearch::local_dominance_check_no_dr(const NodePtr &node_ptr) {
  auto pareto_list_id = pareto_list[node_ptr->id];

  for (auto &pareto_list_element: pareto_list_id) {
    if (pareto_list_element->g[0] <= node_ptr->g[0] &&
      pareto_list_element->g[1] <= node_ptr->g[1]) {
      return true;
    }
  }
  return false;
  // const auto it = pareto_list_id.lower_bound(node_ptr);
  // if (it == pareto_list_id.end()) {
  //   auto prev_it = std::prev(it);
  //   assert((*prev_it)->g[0] <= node_ptr->g[0]);
  //   return ((*prev_it)->g[1] <= node_ptr->g[1]);
  // }
  // if ((*it)->g ==
  //     node_ptr->g) { // there is a value dominating the current value
  //   return true;
  // }
  // if (it == pareto_list_id.begin()) {
  //   assert((*it)->g[0] > node_ptr->g[0] ||
  //          ((*it)->g[0] == node_ptr->g[0]) && (*it)->g[1] > node_ptr->g[1]);
  //   return false;
  // }
  //
  // auto prev_it = std::prev(it);
  // assert((*it)->g[0] > node_ptr->g[0] ||
  //        ((*it)->g[0] == node_ptr->g[0]) && (*it)->g[1] > node_ptr->g[1]);
  // assert((*prev_it)->g[0] <= node_ptr->g[0]);
  // return (*prev_it)->g[1] <= node_ptr->g[1];
}

/**
 * See if the current node is globally dominated by an existing solution of
 * target
 *
 * @param node_ptr the node to check global dominance check for
 * @param solutions the solutions set
 * @return True if the node is globally dominated, else False
 */
bool ForwardSearch::global_dominance_check(const NodePtr &node_ptr,
                                           const SolutionSet &solutions) {

  for (auto &sol: solutions) {
    if (sol->g[0] <= node_ptr->f[0] && sol->g[1] <= node_ptr->f[1]) {
      return false;
    }
  }
  return true;
  // }
  //
  // const auto it = pareto_list_id.lower_bound(node_ptr);
  // if (it == pareto_list_id.end()) {
  //   auto prev_it = std::prev(it);
  //   return ((*prev_it)->g[1] <= node_ptr->f[1]);
  // }
  // if ((*it)->g == node_ptr->g) { // there exists a solution with the same
  // value
  //   return true;
  // }
  // if (it == pareto_list_id.begin()) { // no dominating vlaue
  //   return false;
  // }
  // // else, perform dominance check with DR
  // auto prev_it = std::prev(it);
  // return ((*prev_it)->g[1] <= node_ptr->f[1]);
}

/**
 * comparator used for comparing heuristic values by reverse
 *
 * @param a first value to compare
 * @param b second value to compare
 * @return True if a is reverse-lexicographically greater than b, else False
 */
bool ForwardSearch::comp(const std::vector<float> &a,
                         const std::vector<float> &b) {
  return a[1] > b[1] || (a[1] == b[1] && a[0] > b[0]);
}

/**
 * find the lexicographically first heuristic value that together with the g-
 * value is not dominated by an existing solution of target
 *
 * @param g_value current g-value of a node
 * @param target the target id
 * @param node_mvh the heuristic values of the node
 * @return first globally-undominated heuristic value
 */
std::optional<std::vector<float>>
ForwardSearch::get_first_undominated_heuristic_value(
    const std::vector<float> &g_value, const size_t &target,
    const std::vector<std::vector<float>> &node_mvh) {
  for (auto &heuristic: node_mvh) {
    std::vector new_value = {heuristic[0] + g_value[0], heuristic[1] + g_value[1]};
    if (!(new_value[0] >= min_g2[target][0] && new_value[1] >=min_g2[target][1])) {
      // std::cout << new_value[0] << ", " << new_value[1] << "  " << min_g2[target][0] << ',' << min_g2[target][1] << std::endl;
      return std::optional{heuristic};
    }
  }
  return std::nullopt;

  // Use upper_bound with the custom comparator to find the position
  if (min_g2[target][1] <= g_value[1]) {
    return std::nullopt;
  }
  const std::vector v = {static_cast<float>(MAX_COST),
                         static_cast<float>(MAX_COST)};
  if (min_g2[target] == v) { // if target was not encountered, return the first
                             // heuristic value in the mvh list
    return std::optional{node_mvh[0]};
  }
  const std::vector value_to_compare = {min_g2[target][0] - g_value[0],
                                        min_g2[target][1] - g_value[1]};
  // TODO - something feels a bit off here...
  const auto upper = std::upper_bound(node_mvh.begin(), node_mvh.end(),
                                      value_to_compare, comp);
  // we need the first element that h[1] + g[1] will be less than target g[1].
  // which means, we need the element "right" to the desired location to insert
  // g-value, assuming we would have wanted to insert it by it's second vector
  // entry.

  // std::cout << "f-check: ";
  // for (auto &single_mvh: node_mvh) {
  // std::cout << "(" << single_mvh[0] << "," << single_mvh[1] << "), ";
  // }
  // std::cout << std::endl;

  if (upper == node_mvh.end()) {
    // If target is less than all elements, return None to indicate that there
    // is no relevant value for the heuristic
    // std::cout << "  no chosen heuristic, target is (" << min_g2[target][0] <<
    // "," << min_g2[target][1] << ")" << std::endl; std::cout << "  g-value ("
    // << g_value[0] << "," << g_value[1] << ") " << std::endl;

    return std::nullopt;
  }
  // else, return the value derived from the upper bound
  // std::cout << "  chosen heuristic (" << (*upper)[0] << "," << (*upper)[1] <<
  // ") , target is (" << min_g2[target][0] << "," << min_g2[target][1] << ")"
  // << std::endl; std::cout << "  g-value (" << g_value[0] << "," << g_value[1]
  // << ") " << std::endl;

  return std::optional{(*upper)};
}

/**
 * find the lexicographically first heuristic value that together with the g-
 * value is not dominated by an existing solution of target
 *
 * @param g_value current g-value of a node
 * @param target the target id
 * @param node_mvh the heuristic values of the node
 * @return first globally-undominated heuristic value
 */
std::optional<std::vector<float>>
ForwardSearch::get_first_undominated_heuristic_value_naive(
    const std::vector<float> &g_value, const size_t &target,
    const std::vector<std::vector<float>> &node_mvh) {
  // Use upper_bound with the custom comparator to find the position
  if (min_g2[target][1] <= g_value[1]) {
    return std::nullopt;
  }
  for (auto &heuristic: node_mvh) {
    std::vector new_value = {heuristic[0] + g_value[0], heuristic[1] + g_value[1]};
    if (!(new_value[0] >= min_g2[target][0] && new_value[1] >=min_g2[target][1])) {
      return std::optional{heuristic};
    }
  }
  return std::nullopt;

  const std::vector v = {static_cast<float>(MAX_COST),
                         static_cast<float>(MAX_COST)};
  if (min_g2[target] == v) { // if target was not encountered, return the first
    // heuristic value in the mvh list
    return std::optional{node_mvh[0]};
  }

  for (auto &h_value: node_mvh) {
    std::vector f_value = {g_value[0] + h_value[0], g_value[1] + h_value[1]};
    if (!(min_g2[target][0] <= f_value[0] and min_g2[target][1] <= f_value[1])) {
      return std::optional{h_value};
    }
    return std::nullopt;
  }
}

bool ForwardSearch::min_based_comp(const std::vector<float> &a,
                                   const std::vector<float> &b) {
  return (a[1] < b[1] || ((a[1] == b[1]) && a[0] < b[0]));
};

void ForwardSearch::operator()(const size_t &source, const size_t &target,
                               const MultiValuedHeuristic &heuristic,
                               SolutionSet &solutions) {
  init_search();
  start_time = std::clock();

  std::priority_queue<NodePtr, std::vector<NodePtr>, CompareNodeByFValue> open;

  auto source_heuristic_value = heuristic[source][0];
  NodePtr source_node = std::make_unique<Node>(source, std::vector<float>(2, 0),
                                               source_heuristic_value);
  open.push(source_node);

  while (!open.empty()) {
    auto node = open.top();
    open.pop();

    num_generation += 1;

    if (local_dominance_check_no_dr(node)) {
      continue;
    }

    if (global_dominance_check(node, solutions)) {
      if (auto new_heuristic_value = get_first_undominated_heuristic_value(
              node->g, target, heuristic[node->id])) {
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
      auto new_h = get_first_undominated_heuristic_value(
          new_g, target, heuristic[outgoing_edge.target]);

      if (new_h == std::nullopt) {
        continue;
      }
      NodePtr successor_node = std::make_shared<Node>(
          outgoing_edge.target, new_g, new_h.value(), node);

      if (local_dominance_check_no_dr(successor_node)) {
        continue;
      }

      open.push(successor_node);
    }
  }
}