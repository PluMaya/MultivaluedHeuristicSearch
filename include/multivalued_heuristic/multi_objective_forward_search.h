//
// Created by crl on 14/12/2024.
//

#ifndef MULTI_OBJECTIVE_FORWARD_SEARCH_H
#define MULTI_OBJECTIVE_FORWARD_SEARCH_H

#include "data_structures/adjacency_matrix.h"
#include "data_structures/apex_path_pair.h"
#include "definitions.h"
#include "solvers/abstract_solver.h"

#include <chrono>
#include <iostream>
#include <optional>
#include <set>

struct CustomComparator {
    bool operator()(const NodePtr& a, const NodePtr& b) const {
        for (int i = 0; i < a->g.size(); i++) {
            if (a->g[i] < b->g[i]) {
                return true;
            }
            if (a->g[i] > b->g[i]) {
                return false;
            }
        }
        return false;
    }
};

// Type alias for the set with the custom comparator
using CustomSet = std::set<NodePtr, CustomComparator>;

class MultiObjectiveForwardSearch {
public:
    std::vector<CustomSet> closed;
    std::vector<std::set<std::vector<float>>> closed_dr;
    size_t runtime{};
    size_t start_time{};
    size_t num_expansion = 0;
    size_t num_generation = 0;
    size_t dominance_check_dr = 0;
    size_t dominance_check_full = 0;
    size_t dominance_check_empty = 0;
    const AdjacencyMatrix& adj_matrix{};
    std::vector<size_t> generated = {};
    std::vector<size_t> expanded = {};

    std::unordered_map<std::string, std::chrono::duration<long long, std::ratio<1, 1000000000>>> time_map;

    void operator()(const size_t& source, const size_t& target,
                    const MultiValuedHeuristic& heuristic,
                    SolutionSet& solutions);

    explicit MultiObjectiveForwardSearch(const AdjacencyMatrix& adj_matrix);

    bool local_dominance_check(const NodePtr& node_ptr);

    bool global_dominance_check(const NodePtr& node_ptr,
                                const size_t& target_id);

    void update_closed_dr(const NodePtr& node_ptr);
    void update_closed(const NodePtr& node_ptr);

    std::optional<std::vector<float>> get_first_undominated_heuristic_value(
        const std::vector<float>& g_value, const size_t& target,
        const std::vector<std::vector<float>>& node_mvh);
};


#endif //MULTI_OBJECTIVE_FORWARD_SEARCH_H
