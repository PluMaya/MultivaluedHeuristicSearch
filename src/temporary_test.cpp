#include "solvers/shortest_path_heuristic_computer.h"

#include <iostream>

#include "multi_objective_parser.h"
#include "multivalued_heuristic/backward_search.h"
#include "multivalued_heuristic/multi_objective_forward_search.h"
#include "multivalued_heuristic/multi_objective_backward_search.h"
#include "solvers/namoa_dr.h"


auto parser = MultiObjectiveParser(
    R"(/mnt/c/Users/crl/CLionProjects/MultivaluedHeuristicSearch/resources/MNY/NY-road-d-t-l.txt)");
int num_objectives = 3;
auto adjecency_matrix = parser.parse_graph(num_objectives);
float eps = 0.1;
const size_t source = 83688;
const size_t target = 146074;

void test_multi_objective_forward_search_doesnt_crash() {

    std::cout << "finished loading graph" << std::endl;

    Heuristic source_heuristic = ShortestPathHeuristicComputer::compute_ideal_point_heuristic(
        source, adjecency_matrix);
    Heuristic target_heuristic = ShortestPathHeuristicComputer::compute_ideal_point_heuristic(
        target, adjecency_matrix);

	UpperBoundHeuristic target_heuristic_with_bound = ShortestPathHeuristicComputer::compute_ideal_point_heuristic_with_bound(
        target, adjecency_matrix);
	auto general_upper_bound = target_heuristic_with_bound(source);
	std::vector<float> bound = std::vector<float>(num_objectives, 0);
	for (int i = 0; i < num_objectives; i++) {
		for (int j = 0; j < num_objectives; j++) {
	        bound[i] = std::max(bound[i], general_upper_bound[j][i]);
        }
	}
    std::cout << "finished creating heuristic" << std::endl;
    // auto bs = BackwardSearch(adjecency_matrix, EPS(num_objectives, eps));
    // auto bs_result = bs(source, target, target_heuristic, source_heuristic,
    //     true);
    // std::cout << "total time: " << bs.runtime << std::endl;
    // std::cout << "num expansions: " << bs.num_expansion << std::endl;
    // std::cout << "num generations " << bs.num_generation << std::endl;

    auto mo_bs = MultiObjectiveBackwardSearch(adjecency_matrix, EPS(num_objectives, eps));
    auto result = mo_bs(source, target, target_heuristic, source_heuristic, bound);
    std::cout << "total time: " << mo_bs.runtime << std::endl;
    std::cout << "num expansions: " << mo_bs.num_expansion << std::endl;
    std::cout << "num generations " << mo_bs.num_generation << std::endl;

    std::cout << "finished running multi objective backward search" << std::endl;

    auto mo_fs = MultiObjectiveForwardSearch(adjecency_matrix);
    SolutionSet solutions;
    mo_fs(target, source, result, solutions);
    std::cout << "finished running multi objective forward search" << std::endl;

    std::cout << "total time: " << mo_fs.runtime << std::endl;
    std::cout << "num expansions: " << mo_fs.num_expansion << std::endl;
    std::cout << "num generations " << mo_fs.num_generation << std::endl;

    std::cout << "found " << solutions.size() << " solutions" << std::endl;
    // for (const auto& sol : solutions) {
    //     std::cout << "g=(";
    //     for (float i : sol->g) {
    //         std::cout << i << ",";
    //     }
    //     std::cout << ")" << std::endl;
    // }
}

void test_namoa_doesnt_crash() {
  Heuristic h = ShortestPathHeuristicComputer::compute_ideal_point_heuristic(
      source, adjecency_matrix);

  std::cout << "finished creating heuristic" << std::endl;
  auto namoa_dr = NAMOAdr(adjecency_matrix);
  std::vector<NodePtr> solutions;
  namoa_dr(target, source, h, solutions, 0);
  std::cout << "total time: " << namoa_dr.runtime << std::endl;
  std::cout << "num expansions: " << namoa_dr.num_expansion << std::endl;
  std::cout << "num generations " << namoa_dr.num_generation << std::endl;

  std::cout << "finished running NAMOAdr" << std::endl;
  std::cout << "found " << solutions.size() << " solutions" << std::endl;
  for (const auto& sol: solutions) {
    std::cout << "g=(";
    for (int i = 0; i < sol->g.size(); i++) {
      std::cout << sol->g[i] << ",";
    }
    std::cout << ")" << std::endl;
  }
}


int main(int argc, char* argv[]) {
    test_namoa_doesnt_crash();
    test_multi_objective_forward_search_doesnt_crash();
}
