//
// Created by crl on 17/07/2024.
//

#include "solvers/shortest_path_heuristic_computer.h"

#include "data_structures/node.h"

#include <iostream>
#include <queue>

struct PairComparator {
  bool operator()(const std::pair<float, size_t> &lhs,
                  const std::pair<float, size_t> &rhs) const {
    if (lhs.first != rhs.first) {
      return lhs.first >
             rhs.first; // Compare by the first element (float), smallest first
    }
    return lhs.second >
           rhs.second; // If first elements are equal, compare by the second
                       // element (size_t), smallest first
  }
};

static std::vector<float>
compute_single_cost(const size_t &source,
                    const AdjacencyMatrix &adjacency_matrix,
                    const int cost_idx) {
  size_t V = adjacency_matrix.size() + 1;
  std::vector<float> dist(V, static_cast<float>(INT_MAX));
  std::vector<size_t> parent(V, -1);
  std::priority_queue<std::pair<float, size_t>,
                      std::vector<std::pair<float, size_t>>, PairComparator>
      pq;

  dist[source] = 0;
  pq.emplace(0, source);

  while (!pq.empty()) {
    size_t u = pq.top().second;
    pq.pop();

    for (auto edge : adjacency_matrix[u]) {
      float move_cost = dist[edge.source] + edge.cost.at(cost_idx);
      if (move_cost < INT_MAX && move_cost < dist[edge.target]) {
        dist[edge.target] = move_cost;
        pq.emplace(dist[edge.target], edge.target);
        parent[edge.target] = u;
      }
    }
  }
  return dist;
}

Heuristic ShortestPathHeuristicComputer::compute_ideal_point_heuristic(
    const size_t &source, const AdjacencyMatrix &adjacency_matrix) {
  std::vector<std::vector<float>> heuristic_values_for_objectives(
      adjacency_matrix.num_of_objectives);

  for (int j = 0; j < adjacency_matrix.num_of_objectives; j++) {
    heuristic_values_for_objectives[j] =
        compute_single_cost(source, adjacency_matrix, j);
  }

  return [heuristic_values_for_objectives](size_t index) -> std::vector<float> {
    std::vector<float> result;
    result.reserve(heuristic_values_for_objectives.size());
    for (const auto &heuristic_values_for_objective :
         heuristic_values_for_objectives) {
      result.push_back(heuristic_values_for_objective[index]);
    }
    return result;
  };
}
