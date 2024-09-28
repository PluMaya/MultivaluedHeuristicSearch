//
// Created by crl on 23/07/2024.
//

#include "definitions.h"
#include "multivalued_heuristic/backward_search.h"
#include "multivalued_heuristic/experimental_forward_search.h"
#include "parser.h"
#include "solvers/shortest_path_heuristic_computer.h"

#include <iostream>
void test_forward_search() {
  std::cout << "-------------------------------" << std::endl;
  auto parser = Parser();
  auto adjecency_matrix = parser.default_graph();
  std::cout << "finished loading graph" << std::endl;
  constexpr size_t source = 216144;
  constexpr size_t target = 150000;
  Heuristic blind_heuristic = [](size_t vertex) -> std::vector<float> {
    return {0,0};
  };
  Heuristic to_target =
      ShortestPathHeuristicComputer::compute_ideal_point_heuristic(
          target, adjecency_matrix);
  Heuristic to_source =
      ShortestPathHeuristicComputer::compute_ideal_point_heuristic(
          source, adjecency_matrix);
  std::cout << "finished creating heuristic" << std::endl;
  std::cout << "-------------------------------" << std::endl;

  // auto backward_search2 =BackwardSearch(adjecency_matrix, {0.1,0.1});
  // MultiValuedHeuristic mvh2 =
  //   backward_search2(source, target, to_target, to_source);
  // auto res2 = mvh2(target);
  // for (auto &sol2: res2) {
  //   std::cout << "(" << sol2[0] << "," << sol2[1] << "), ";
  // }
  // std::cout << std::endl;

  auto backward_search = BackwardSearch(adjecency_matrix, {0.1, 0.1});
  MultiValuedHeuristic mvh =
      backward_search(source, target, to_target, to_source);

  for (int i = 0; i < adjecency_matrix.size(); i+=1) {
    auto state_mvh  = mvh(i);
    auto ideal_point_h = to_source(i);
    std::vector<float> to_compare = {state_mvh[0][0], state_mvh[state_mvh.size()-1][1]};
    if ((to_compare[0] > ideal_point_h[0]) or (to_compare[1] > ideal_point_h[1]) ){
      std::cout << to_compare[0] << ", " << to_compare[1] << "  " << ideal_point_h[0] << ", " << ideal_point_h[1] << std::endl;
      std::cout << "problem with state " << i << std::endl;
    }
  }

  // MultiValuedHeuristic mvh = [to_source](size_t vertex) -> std::vector<std::vector<float>> {
  //   return {to_source(vertex)};
  // };
  auto res = mvh(target);
  std::cout << "num solutions: " << res.size() << std::endl;
  // for (auto &sol: res) {
  //   std::cout << "(" << sol[0] << "," << sol[1] << "), ";
  // }
  std::cout << std::endl;


  std::cout << "total time: " << (std::clock() - backward_search.start_time)
            << std::endl;
  std::cout << "num expansions: " << backward_search.num_expansion << std::endl;
  std::cout << "num generations " << backward_search.num_generation
            << std::endl;
  std::cout << "finished running backward_search" << std::endl;
  std::cout << "total heuristic values of target: " << res.size() << std::endl;
  std::cout << "-------------------------------" << std::endl;

  std::cout << "starting forward search" << std::endl;
  std::vector<NodePtr> solutions;
  auto forward_search = ExperimentalForwardSearch(adjecency_matrix, {0, 0});
  forward_search(target, source, mvh, solutions);
  std::cout << "total time: " << (std::clock() - forward_search.start_time)
            << std::endl;

  std::cout << "num expansions: " << forward_search.num_expansion << std::endl;
  std::cout << "num generations " << forward_search.num_generation << std::endl;
  std::cout << "found " << solutions.size() << " solutions" << std::endl;
  for (const auto &sol : solutions) {
    std::cout << "g=(" << sol->g[0] << "," << sol->g[1] << ")" << std::endl;
  }
}

int main() {
  test_forward_search();
  return 0;
}
