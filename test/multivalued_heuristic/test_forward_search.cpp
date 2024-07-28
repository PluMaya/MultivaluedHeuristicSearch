//
// Created by crl on 23/07/2024.
//

#include "definitions.h"
#include "multivalued_heuristic/backward_search.h"
#include "multivalued_heuristic/forward_search.h"
#include "parser.h"
#include "solvers/shortest_path_heuristic_computer.h"

#include <iostream>
void test_forward_search() {
  auto parser = Parser();
  auto adjecency_matrix = parser.default_graph();
  std::cout << "finished loading graph" << std::endl;
  constexpr size_t source = 216144;
  constexpr size_t target = 150000;
  Heuristic to_target =
      ShortestPathHeuristicComputer::compute_ideal_point_heuristic(
          target, adjecency_matrix);
  Heuristic to_source =
      ShortestPathHeuristicComputer::compute_ideal_point_heuristic(
          source, adjecency_matrix);
  std::cout << "finished creating heuristic" << std::endl;

  auto backward_search = BackwardSearch(adjecency_matrix, {0.0, 0.0});
  MultiValuedHeuristic mvh =
      backward_search(source, target, to_target, to_source);
  std::cout << "total time: " << (std::clock() - backward_search.start_time)
            << std::endl;
  std::cout << "num expansions: " << backward_search.num_expansion << std::endl;
  std::cout << "num generations " << backward_search.num_generation
            << std::endl;
  std::cout << "finished running backward_search" << std::endl;
  auto res = mvh(target);
  std::cout << "total heuristic values of target: " << res.size() << std::endl;

  std::cout << "starting forward search" << std::endl;
  std::vector<NodePtr> solutions;
  auto forward_search = ForwardSearch(adjecency_matrix, {0, 0});
  forward_search(target, source, mvh, solutions);
  std::cout << "total time: " << (std::clock() - forward_search.start_time);
  std::cout << "found " << solutions.size() << " solutions" << std::endl;
  for (const auto& sol: solutions) {
    std::cout << "g=(" << sol->g[0] << "," << sol->g[1] << ")" << std::endl;
  }
}

int main() {
  test_forward_search();
  return 0;
}
