//
// Created by crl on 30/11/2024.
//

#ifndef EDGE_BASED_BACKWARD_SEARCH_H
#define EDGE_BASED_BACKWARD_SEARCH_H



#include <ctime>
#include <map>

#include "data_structures/adjacency_matrix.h"
#include "data_structures/apex_path_pair.h"
#include "data_structures/map_queue.h"
#include "data_structures/node.h"
#include "definitions.h"
#include "solvers/apex.h"


using MVHResults =  std::vector<std::map<size_t, std::vector<float>>>;

class EdgeBasedBackwardSearch {
public:
    const AdjacencyMatrix& adj_matrix;

    std::clock_t start_time = std::clock();
    size_t num_expansion = 0;
    size_t num_generation = 0;
    bool global_stop_condition = true;
    float runtime = 0.0f;

    std::vector<float> min_g2;
    std::vector<std::vector<ApexPathPairPtr>> expanded;

    NodePtr last_solution = nullptr;

    MultiValuedHeuristic operator()(const size_t& source, const size_t& target,
                                    const Heuristic& heuristic_to_target,
                                    const Heuristic& heuristic_to_source,
                                    bool global_stop_condition = true);

    virtual ~EdgeBasedBackwardSearch() = default;

    [[nodiscard]] bool local_dominance_check(const NodePtr& cur_node) const {
        return (cur_node->g[1] >= min_g2[cur_node->id]);
    }

    [[nodiscard]] bool global_dominance_check(const NodePtr& cur_node, const size_t& target) const;
    bool is_child_locally_dominated(const NodePtr& parent, const Edge& edge);
    bool is_child_globally_dominated(const NodePtr& parent, const Edge& edge,
                                     const std::vector<float>& h);

    explicit EdgeBasedBackwardSearch(const AdjacencyMatrix& adj_matrix);

    [[nodiscard]] std::vector<std::vector<float>> make_list_of_values(const std::map<size_t, std::vector<float>>& node_frontier,
                                                        const std::vector<float>& heuristic_value) const;

    void set_min_g2(const NodePtr& cur_node) {
        min_g2[cur_node->id] = cur_node->g[1];
    }
};

#endif //EDGE_BASED_BACKWARD_SEARCH_H
