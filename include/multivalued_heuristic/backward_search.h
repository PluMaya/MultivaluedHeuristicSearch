//
// Created by crl on 22/07/2024.
//

#ifndef BACKWARD_SEARCH_H
#define BACKWARD_SEARCH_H

#include <ctime>

#include "data_structures/adjacency_matrix.h"
#include "data_structures/apex_path_pair.h"
#include "data_structures/map_queue.h"
#include "data_structures/node.h"
#include "definitions.h"
#include "solvers/apex.h"


#include <unordered_map>

using BackwardSearchSolutionSet = std::unordered_map<size_t, ApexSolutionSet>;

class BackwardSearch {
public:
    const AdjacencyMatrix& adj_matrix;
    EPS eps;

    std::clock_t start_time = std::clock();
    size_t num_expansion = 0;
    size_t num_generation = 0;
    bool global_stop_condition = true;
    float runtime = 0.0f;

    std::vector<float> min_g2;
    std::vector<std::vector<ApexPathPairPtr>> expanded;

    ApexPathPairPtr last_solution = nullptr;

    MultiValuedHeuristic operator()(const size_t& source, const size_t& target,
                                    const Heuristic& heuristic_to_target,
                                    const Heuristic& heuristic_to_source,
                                    bool global_stop_condition = true);
    virtual ~BackwardSearch() = default;

    virtual void insert(ApexPathPairPtr& ap, MapQueue& queue);
    [[nodiscard]] bool is_dominated(const ApexPathPairPtr& ap) const;

    [[nodiscard]] bool local_dominance_check(const ApexPathPairPtr& ap) const {
        return (ap->apex->g[1] >= min_g2[ap->id]);
    }

    [[nodiscard]] bool global_dominance_check(const ApexPathPairPtr& ap) const;
    bool is_child_locally_dominated(const ApexPathPairPtr& parent, const Edge& edge);
    bool is_child_globally_dominated(const ApexPathPairPtr& parent, const Edge& edge,
                                     const std::vector<float>& h);
    static void update_frontier(const ApexPathPairPtr& ap,
                                ApexSolutionSet& frontier);
    BackwardSearch(const AdjacencyMatrix& adj_matrix, EPS eps);

    std::vector<std::vector<float>> make_list_of_values(const ApexSolutionSet& apex_solution_set,
                    const std::vector<float>& heuristic_value);

    void set_min_g2(const ApexPathPairPtr& ap) {
        min_g2[ap->id] = ap->apex->g[1];
    }
};

#endif // BACKWARD_SEARCH_H
