#include "solvers/shortest_path_heuristic_computer.h"

#include <iostream>

#include "multi_objective_parser.h"
#include "multivalued_heuristic/multi_objective_forward_search.h"
#include "multivalued_heuristic/multi_objective_backward_search.h"

void test_multi_objective_forward_search_doesnt_crash() {
    auto parser = MultiObjectiveParser(
        R"(/mnt/c/Users/crl/CLionProjects/MultivaluedHeuristicSearch/resources/MNY/NY-road-d-t-l.txt)");
    auto adjecency_matrix = parser.parse_graph(2);
    std::cout << "finished loading graph" << std::endl;
    const size_t source = 201401;
    const size_t target = 202000;
    Heuristic source_heuristic = ShortestPathHeuristicComputer::compute_ideal_point_heuristic(
        source, adjecency_matrix);
    Heuristic target_heuristic = ShortestPathHeuristicComputer::compute_ideal_point_heuristic(
        target, adjecency_matrix);

    std::cout << "finished creating heuristic" << std::endl;
    auto mo_bs = MultiObjectiveBackwardSearch(adjecency_matrix, EPS(2, 0));
    auto result = mo_bs(source, target, source_heuristic, target_heuristic, false);
    std::cout << "total time: " << mo_bs.runtime << std::endl;
    std::cout << "num expansions: " << mo_bs.num_expansion << std::endl;
    std::cout << "num generations " << mo_bs.num_generation << std::endl;

    std::cout << "finished running multi objective backward search" << std::endl;

    auto mo_fs = MultiObjectiveForwardSearch(adjecency_matrix);
    SolutionSet solutions;
    mo_fs(target, source, result, solutions);

    std::cout << "found " << solutions.size() << " solutions" << std::endl;
    for (const auto& sol : solutions) {
        std::cout << "g=(";
        for (float i : sol->g) {
            std::cout << i << ",";
        }
        std::cout << ")" << std::endl;
    }
}

int main(int argc, char* argv[]) {
    test_multi_objective_forward_search_doesnt_crash();
}
