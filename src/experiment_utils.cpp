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

void ExperimentUtils::run_boa_star(const AdjacencyMatrix &adjecency_matrix,
                                   const size_t &source, const size_t &target,
                                   const Heuristic &heuristic) {
  auto boa = BOAStar(adjecency_matrix);
  SolutionSet solutions;
  boa(source, target, heuristic, solutions, 300);
  std::cout << "boa"
            << "\t" << source << "\t" << target << "\t" << solutions.size()
            << "\t" << boa.num_expansion << "\t" << boa.num_generation << "\t"
            << boa.runtime << std::endl;
}
void ExperimentUtils::run_apex(const AdjacencyMatrix &adjecency_matrix,
                               const size_t &source, const size_t &target,
                               const EPS &eps, const Heuristic &heuristic) {
  auto apex = ApexSearch(adjecency_matrix, eps);
  SolutionSet solutions;
  apex(source, target, heuristic, solutions, 300);
  std::cout << "apex"
            << "\t" << source << "\t" << target << "\t" << solutions.size()
            << "\t" << apex.num_expansion << "\t" << apex.num_generation << "\t"
            << apex.runtime << std::endl;
}

void ExperimentUtils::run_backward_search(
    const AdjacencyMatrix &adjecency_matrix, const size_t &source,
    const size_t &target, const EPS &eps, const Heuristic &source_to_target,
    const Heuristic &target_to_source) {
  auto backward_search = BackwardSearch(adjecency_matrix, eps);
  MultiValuedHeuristic mvh =
      backward_search(source, target, source_to_target, target_to_source);
  std::cout << "BS"
            << "\t" << source << "\t" << target << "\t" << mvh[target].size()
            << "\t" << backward_search.num_expansion << "\t"
            << backward_search.num_generation << "\t" << backward_search.runtime
            << std::endl;
}

void ExperimentUtils::run_forward_search(
    const AdjacencyMatrix &adjecency_matrix, const size_t &source,
    const size_t &target, const EPS &eps, const MultiValuedHeuristic &mvh) {
  auto forward_search = ExperimentalForwardSearch(adjecency_matrix, eps);
  SolutionSet solutions;
  forward_search(source, target, mvh, solutions);
  std::cout << "FS"
            << "\t" << source << "\t" << target << "\t" << solutions.size()
            << "\t" << forward_search.num_expansion << "\t"
            << forward_search.num_generation << "\t" << forward_search.runtime
            << std::endl;
}

void ExperimentUtils::single_run(AdjacencyMatrix &adjecency_matrix,
                                 size_t source, size_t target,
                                 const std::string &algorithm, const EPS &eps) {
  if (algorithm == "BOA") {
    Heuristic heuristic =
        ShortestPathHeuristicComputer::compute_ideal_point_heuristic(
            target, adjecency_matrix);
    run_boa_star(adjecency_matrix, source, target, heuristic);
  } else if (algorithm == "APEX") {
    Heuristic heuristic =
        ShortestPathHeuristicComputer::compute_ideal_point_heuristic(
            target, adjecency_matrix);
    run_apex(adjecency_matrix, source, target, eps, heuristic);
  } else if (algorithm == "BS") {
    Heuristic source_to_target =
        ShortestPathHeuristicComputer::compute_ideal_point_heuristic(
            target, adjecency_matrix);
    Heuristic target_to_sourfce =
        ShortestPathHeuristicComputer::compute_ideal_point_heuristic(
            source, adjecency_matrix);
    run_backward_search(adjecency_matrix, source, target, eps, source_to_target,
                        target_to_sourfce);
  } else if (algorithm == "FS") {
    Heuristic source_to_target =
        ShortestPathHeuristicComputer::compute_ideal_point_heuristic(
            target, adjecency_matrix);
    Heuristic target_to_source =
        ShortestPathHeuristicComputer::compute_ideal_point_heuristic(
            source, adjecency_matrix);
    auto backward_search = BackwardSearch(adjecency_matrix, eps);
    // See that in order to compute the mvh for forward search, the source is
    // actually the target of the backward search and the target is actually the
    // source!
    MultiValuedHeuristic mvh =
        backward_search(target, source, target_to_source, source_to_target);
    run_forward_search(adjecency_matrix, source, target, eps, mvh);
  } else {
    std::cout << "Unknown algorithm: " << algorithm << std::endl;
  }
}
