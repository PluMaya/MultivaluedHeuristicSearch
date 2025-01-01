// //
// // Created by crl on 15/07/2024.
// //
//
// #include "solvers/shortest_path_heuristic_computer.h"
//
// #include <iostream>
//
// #include "multi_objective_parser.h"
// #include "solvers/namoa_dr.h"
//
// void test_namoa_doesnt_crash() {
//   auto parser = MultiObjectiveParser(R"(/mnt/c/Users/crl/CLionProjects/MultivaluedHeuristicSearch/resources/MNY/NY-road-d-t-l.txt)");
//   auto adjecency_matrix = parser.parse_graph(2);
//   std::cout <<"finished loading graph" << std::endl;
//   const size_t source = 201401;
//   const size_t target = 202000;
//   Heuristic h = ShortestPathHeuristicComputer::compute_ideal_point_heuristic(
//       target, adjecency_matrix);
//
//   std::cout << "finished creating heuristic" << std::endl;
//   auto namoa_dr = NAMOAdr(adjecency_matrix);
//   std::vector<NodePtr> solutions;
//   namoa_dr(source, target, h, solutions, 300);
//   std::cout << "total time: " << namoa_dr.runtime << std::endl;
//   std::cout << "num expansions: " << namoa_dr.num_expansion << std::endl;
//   std::cout << "num generations " << namoa_dr.num_generation << std::endl;
//
//   std::cout << "finished running NAMOAdr" << std::endl;
//   std::cout << "found " << solutions.size() << " solutions" << std::endl;
//   for (const auto& sol: solutions) {
//     std::cout << "g=(";
//     for (int i = 0; i < sol->g.size(); i++) {
//       std::cout << sol->g[i] << ",";
//     }
//     std::cout << ")" << std::endl;
//   }
// }
//
// int main(int argc, char* argv[]) {
//   test_namoa_doesnt_crash();
//
// }
