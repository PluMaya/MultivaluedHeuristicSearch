//
// Created by crl on 13/12/2024.
//

#ifndef MULTI_OBJECTIVE_BACKWARD_SEARCH_H
#define MULTI_OBJECTIVE_BACKWARD_SEARCH_H
#include <set>

#include "definitions.h"
#include "data_structures/adjacency_matrix.h"
#include "data_structures/apex_path_pair.h"
#include "data_structures/map_queue.h"
#include "solvers/apex.h"


class MultiObjectiveBackwardSearch {
public:
    const AdjacencyMatrix& adj_matrix;
    std::clock_t start_time = std::clock();
    bool global_stop_condition{};
    EPS eps;
    size_t num_expansion = 0;
    size_t num_generation = 0;
    size_t runtime{};

    std::vector<std::set<std::vector<float>>> closed_dr;

    void update_close_dr(const ApexPathPairPtr& ap);

    MultiValuedHeuristic operator()(const size_t& source, const size_t& target,
                                    const Heuristic& heuristic_to_target,
                                    const Heuristic& heuristic_to_source,
                                    bool global_stop_condition = true);
    virtual ~MultiObjectiveBackwardSearch() = default;

    [[nodiscard]] bool local_dominance_check(const ApexPathPairPtr& ap) const;
    [[nodiscard]] bool global_dominance_check(const ApexPathPairPtr& ap, std::vector<ApexPathPairPtr> solutions) const;

    MultiObjectiveBackwardSearch(const AdjacencyMatrix& adj_matrix, EPS eps);

    [[nodiscard]] std::vector<std::vector<float>> make_list_of_values(const ApexSolutionSet& apex_solution_set,
                    const std::vector<float>& heuristic_value) const;

};


#endif //MULTI_OBJECTIVE_BACKWARD_SEARCH_H
