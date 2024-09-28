// //
// // Created by crl on 15/07/2024.
// //
//
// #include "parser.h"
// #include "solvers/boa.h"
// #include "solvers/shortest_path_heuristic_computer.h"
//
// #include <iostream>
//
// void test_boa_doesnt_crash() {
//   auto parser = Parser();
//   auto adjecency_matrix = parser.default_graph();
//   std::cout <<"finished loading graph" << std::endl;
//   const size_t source = 216044;
//   const size_t target = 150000;
//   Heuristic h = ShortestPathHeuristicComputer::compute_ideal_point_heuristic(
//       target, adjecency_matrix);
//
//   std::cout << "finished creating heuristic" << std::endl;
//   auto boa = BOAStar(adjecency_matrix);
//   std::vector<NodePtr> solutions;
//   boa(source, target, h, solutions, 300);
//   std::cout << "total time: " << (std::clock() - boa.start_time) << std::endl;
//   std::cout << "num expansions: " << boa.num_expansion << std::endl;
//   std::cout << "num generations " << boa.num_generation << std::endl;
//
//   std::cout << "finished running BOA" << std::endl;
//   std::cout << "found " << solutions.size() << " solutions" << std::endl;
//   for (const auto& sol: solutions) {
//     std::cout << "g=(" << sol->g[0] << "," << sol->g[1] << ")" << std::endl;
//   }
// }
//
//
// int main() {
//   test_boa_doesnt_crash();
//   return 0;
// }
