//
// Created by crl on 28/09/2024.
//

#include "experiment_utils.h"

#include "multivalued_heuristic/backward_search.h"
#include "multivalued_heuristic/experimental_forward_search.h"
#include "solvers/apex.h"
#include "solvers/boa.h"
#include "solvers/shortest_path_heuristic_computer.h"

#include <fstream>
#include <iostream>

void ExperimentUtils::run_boa_star(const AdjacencyMatrix& adjecency_matrix,
                                   const size_t& source, const size_t& target,
                                   const Heuristic& heuristic, const clock_t heuristic_runtime) {
    auto boa = BOAStar(adjecency_matrix);
    SolutionSet solutions;
    boa(source, target, heuristic, solutions, 300);
    std::cout << "BOA"
        << "\t" << source << "\t" << target << "\t" << solutions.size()
        << "\t" << boa.num_expansion << "\t" << boa.num_generation << "\t"
        << boa.runtime << "\t" << heuristic_runtime << std::endl;
}

void ExperimentUtils::run_apex(const AdjacencyMatrix& adjecency_matrix,
                               const size_t& source, const size_t& target,
                               const EPS& eps, const Heuristic& heuristic, const long heuristic_duration) {
    auto apex = ApexSearch(adjecency_matrix, eps);
    SolutionSet solutions;
    apex(source, target, heuristic, solutions, 300);
    std::cout << "Apex"
        << "\t" << source << "\t" << target << "\t" << solutions.size()
        << "\t" << apex.num_expansion << "\t" << apex.num_generation << "\t"
        << apex.runtime << "\t" << heuristic_duration << std::endl;
}

void ExperimentUtils::run_backward_search(
    const AdjacencyMatrix& adjecency_matrix, const size_t& source,
    const size_t& target, const EPS& eps, const Heuristic& source_to_target,
    const Heuristic& target_to_source, bool global_stop_condition) {
    auto backward_search = BackwardSearch(adjecency_matrix, eps);
    MultiValuedHeuristic mvh =
        backward_search(source, target, source_to_target, target_to_source, global_stop_condition);
    std::cout << "BS"
        << "\t" << source << "\t" << target << "\t" << mvh[target].size()
        << "\t" << backward_search.num_expansion << "\t"
        << backward_search.num_generation << "\t" << backward_search.runtime
        << std::endl;
}

void ExperimentUtils::run_forward_search(
    const AdjacencyMatrix& adjecency_matrix, const size_t& source,
    const size_t& target, const EPS& eps, const MultiValuedHeuristic& mvh, const float backward_search_runtime, const
    float source_heuristic_runtime, const float target_heuristic_runtime) {
    auto forward_search = ExperimentalForwardSearch(adjecency_matrix, eps);
    SolutionSet solutions;
    forward_search(source, target, mvh, solutions);
    std::cout << "FS"
        << "\t" << source << "\t" << target << "\t" << solutions.size()
        << "\t" << forward_search.num_expansion << "\t"
        << forward_search.num_generation << "\t" << forward_search.runtime
        << "\t" << backward_search_runtime
        << "\t" << source_heuristic_runtime
        << "\t" << target_heuristic_runtime
        << std::endl;
}

void ExperimentUtils::single_run(AdjacencyMatrix& adjecency_matrix,
                                 size_t source, size_t target,
                                 const std::string& algorithm, const EPS& eps, bool global_stop_condition,
                                 std::vector<int> multi_sources) {
    if (multi_sources.empty()) {
        multi_sources.push_back(source);
    }
    if (algorithm == "BOA") {
        auto heuristic_start_time = std::clock();
        Heuristic heuristic =
    ShortestPathHeuristicComputer::compute_ideal_point_heuristic(
        target, adjecency_matrix);
        auto heuristic_duration = static_cast<long>(std::clock() - heuristic_start_time);
        for (auto multi_source: multi_sources) {
            run_boa_star(adjecency_matrix, multi_source, target, heuristic, heuristic_duration);
        }
    }
    else if (algorithm == "APEX") {
        auto start_time = std::clock();
        Heuristic heuristic =
            ShortestPathHeuristicComputer::compute_ideal_point_heuristic(
                target, adjecency_matrix);
        auto heuristic_duration = static_cast<long>(std::clock() - start_time);
        for (auto multi_source: multi_sources) {
            run_apex(adjecency_matrix, multi_source, target, eps, heuristic, heuristic_duration);
        }
    }
    else if (algorithm == "BS") {
        if (global_stop_condition == false) {
            Heuristic blind_heuristic = [](size_t vertex) -> std::vector<float> {
                return {0, 0};
            };
            run_backward_search(adjecency_matrix, source, target, eps, blind_heuristic,
                                blind_heuristic, global_stop_condition);
        }
        else {
            Heuristic source_to_target =
                ShortestPathHeuristicComputer::compute_ideal_point_heuristic(target, adjecency_matrix);
            Heuristic target_to_source =
                ShortestPathHeuristicComputer::compute_ideal_point_heuristic(source, adjecency_matrix);
            run_backward_search(adjecency_matrix, source, target, eps, source_to_target,
                                target_to_source, global_stop_condition);
        }
    }
    else if (algorithm == "FS") {
        if (global_stop_condition == false) {
            Heuristic blind_heuristic = [](size_t vertex) -> std::vector<float> {
                return {0, 0};
            };
            auto backward_search = BackwardSearch(adjecency_matrix, eps);
            // See that in order to compute the mvh for forward search, the source is
            // actually the target of the backward search and the target is actually the
            // source!
            MultiValuedHeuristic mvh =
                backward_search(target, source, blind_heuristic, blind_heuristic, global_stop_condition);

            if (multi_sources.size() != 0) {
                for (auto multi_source : multi_sources) {
                    run_forward_search(adjecency_matrix, multi_source, target, eps, mvh, backward_search.runtime,
                                       0, 0);
                }
            }
            else {
                run_backward_search(adjecency_matrix, source, target, eps, blind_heuristic,
                                    blind_heuristic, global_stop_condition);
            }
        }
        else {
            if (not multi_sources.size() == 1) {
                std::cerr << "FS can only be run with a single source when global stop condition is true"
                    << std::endl;
            }
            auto start_time = std::clock();
            Heuristic source_to_target =
                ShortestPathHeuristicComputer::compute_ideal_point_heuristic(
                    target, adjecency_matrix);
            auto source_heuristic_runtime = static_cast<float>(std::clock() - start_time);
            start_time = std::clock();
            Heuristic target_to_source =
                ShortestPathHeuristicComputer::compute_ideal_point_heuristic(
                    source, adjecency_matrix);
            auto target_heuristic_runtime = static_cast<float>(std::clock() - start_time);
            auto backward_search = BackwardSearch(adjecency_matrix, eps);
            // See that in order to compute the mvh for forward search, the source is
            // actually the target of the backward search and the target is actually the
            // source!
            MultiValuedHeuristic mvh =
                backward_search(target, source, target_to_source, source_to_target);

            run_forward_search(adjecency_matrix, source, target, eps, mvh, backward_search.runtime,
                               source_heuristic_runtime, target_heuristic_runtime);
        }
    }
    else {
        std::cout << "Unknown algorithm: " << algorithm << std::endl;
    }
}
