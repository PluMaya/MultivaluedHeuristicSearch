// //
// // Created by crl on 20/07/2024.
// //
//
// #include "definitions.h"
// #include "parser.h"
// #include "solvers/apex.h"
// #include "solvers/shortest_path_heuristic_computer.h"
//
// #include <iostream>
// void test_apex_doesnt_crash() {
//   auto parser = Parser();
//   auto adjecency_matrix = parser.default_graph();
//   std::cout << "finished loading graph" << std::endl;
//   const size_t source = 216144;
//   const size_t target = 150000;
//   Heuristic h = ShortestPathHeuristicComputer::compute_ideal_point_heuristic(
//       target, adjecency_matrix);
//   std::cout << "finished creating heuristic" << std::endl;
//
//   auto apex = ApexSearch(adjecency_matrix, {0.1, 0.1});
//   std::vector<NodePtr> solutions;
//   apex(source, target, h, solutions, 300);
//   std::cout << "total time: " << (std::clock() - apex.start_time) << std::endl;
//   std::cout << "num expansions: " << apex.num_expansion << std::endl;
//   std::cout << "num generations " << apex.num_generation << std::endl;
//   std::cout << "finished running Apex" << std::endl;
//   std::cout << "found " << solutions.size() << " solutions" << std::endl;
//   for (const auto &sol : solutions) {
//     std::cout << "f=(" << sol->f[0] << "," << sol->f[1] << ")"
//               << "g=(" << sol->g[0] << "," << sol->g[1] << ")" << std::endl;
//   }
// }
//
// int main() {
//   test_apex_doesnt_crash();
//   return 0;
// }
