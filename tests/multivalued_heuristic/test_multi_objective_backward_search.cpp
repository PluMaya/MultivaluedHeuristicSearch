//
// Created by crl on 15/07/2024.
//

#include "solvers/shortest_path_heuristic_computer.h"

#include <iostream>

#include "multi_objective_parser.h"
#include "multivalued_heuristic/multi_objective_backward_search.h"

void test_multi_objective_backward_search_doesnt_crash() {
    auto parser = MultiObjectiveParser(
        R"(/mnt/c/Users/crl/CLionProjects/MultivaluedHeuristicSearch/resources/MNY/NY-road-d-t-l.txt)");
    int num_objectives = 2;
    auto adjecency_matrix = parser.parse_graph(num_objectives);
    std::cout << "finished loading graph" << std::endl;
    const size_t source = 201401;
    const size_t target = 202000;
    Heuristic source_heuristic = ShortestPathHeuristicComputer::compute_ideal_point_heuristic(
        source, adjecency_matrix);
    Heuristic target_heuristic = ShortestPathHeuristicComputer::compute_ideal_point_heuristic(
        target, adjecency_matrix);

    std::cout << "finished creating heuristic" << std::endl;
    auto mo_bs = MultiObjectiveBackwardSearch(adjecency_matrix, EPS(num_objectives, 0));
    auto result = mo_bs(source, target, target_heuristic, source_heuristic,
        true);
    std::cout << "total time: " << mo_bs.runtime << std::endl;
    std::cout << "num expansions: " << mo_bs.num_expansion << std::endl;
    std::cout << "num generations " << mo_bs.num_generation << std::endl;

    std::cout << "finished running multi objective backward search" << std::endl;
    std::cout << "found " << result[target].size() << " solutions" << std::endl;
    for (const auto& sol : result[target]) {
        std::cout << "g=(";
        for (int i = 0; i < sol.size(); i++) {
            std::cout << sol[i] << ",";
        }
        std::cout << ")" << std::endl;
    }
}

int main(int argc, char* argv[]) {
    test_multi_objective_backward_search_doesnt_crash();
}
