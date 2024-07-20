//
// Created by crl on 13/07/2024.
//

#ifndef SHORTEST_PATH_HEURISTIC_COMPUTER_H
#define SHORTEST_PATH_HEURISTIC_COMPUTER_H
#include "../data_structures/adjacency_matrix.h"
#include "../definitions.h"

class ShortestPathHeuristicComputer {
public:
  static Heuristic compute_ideal_point_heuristic(const size_t &target,
    const AdjacencyMatrix &adjacency_matrix);
};

#endif // SHORTEST_PATH_HEURISTIC_COMPUTER_H
