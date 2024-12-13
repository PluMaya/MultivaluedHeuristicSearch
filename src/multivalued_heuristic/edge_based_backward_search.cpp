//
// Created by crl on 22/07/2024.
//

#include <fstream>
#include <iostream>
#include <multivalued_heuristic/edge_based_backward_search.h>

#include <utility>
#include <boost/program_options/detail/cmdline.hpp>

bool EdgeBasedBackwardSearch::global_dominance_check(const NodePtr& cur_node, const size_t& target) const {
    if (global_stop_condition == false) {
        return false;
    }
    if (last_solution == nullptr) {
        return false;
    }
    // Dominance check
    return cur_node->f[1] >= min_g2[target];
}

EdgeBasedBackwardSearch::EdgeBasedBackwardSearch(const AdjacencyMatrix& adj_matrix)
    : adj_matrix(adj_matrix) {
    expanded.resize(adj_matrix.size() + 1);
    min_g2.resize(adj_matrix.size() + 1, static_cast<float>(MAX_COST));
}

bool EdgeBasedBackwardSearch::is_child_locally_dominated(const NodePtr& parent, const Edge& edge) {
    return parent->g[1] + edge.cost[1] >= min_g2[edge.target];
}

bool EdgeBasedBackwardSearch::is_child_globally_dominated(const NodePtr& parent, const Edge& edge,
                                                          const std::vector<float>& h) {
    if (global_stop_condition == false) {
        return false;
    }
    if (last_solution == nullptr) {
        return false;
    }

    std::vector f_value = {
        parent->g[0] + edge.cost[0] + h[0],
        parent->g[1] + edge.cost[1] + h[1]
    };

    // apex is not bunded
    return (last_solution->f[0] > f_value[0] or last_solution->f[1] > f_value[1]);
}

std::vector<std::vector<float>>
EdgeBasedBackwardSearch::make_list_of_values(const std::map<size_t, std::vector<float>>& node_frontier,
                                             const std::vector<float>& heuristic_value) const {
    std::vector<std::vector<float>> result = {};
    if (node_frontier.empty() || node_frontier.size() == 1) {
        result.push_back(heuristic_value);
    }
    else {
        for (const auto& [key, value] : node_frontier) {
            result.push_back(value);
        }
        std::sort(result.begin(), result.end());
        std::vector<std::vector<float>> new_result = {};
        new_result.push_back(result[0]);
        auto current_value = result[0];
        for (auto i = 1; i < result.size(); i++) {
            if (current_value[0] < result[i][0] and current_value[1] < result[i][1]) {
                continue;
            }
            new_result.push_back(result[i]);
            current_value = result[i];
        }
        if (global_stop_condition == true) {
            result.at(0).at(0) = heuristic_value.at(0);
            result.at(result.size() - 1).at(1) = heuristic_value.at(1);
        }
    }
    return result;
}

MultiValuedHeuristic
EdgeBasedBackwardSearch::operator()(const size_t& source, const size_t& target,
                                    const Heuristic& heuristic_to_target,
                                    const Heuristic& heuristic_to_source,
                                    bool global_stop_condition) {
    start_time = std::clock();
    this->global_stop_condition = global_stop_condition;

    MVHResults frontiers(adj_matrix.size() + 1);

    std::priority_queue<NodePtr, std::vector<NodePtr>, CompareNodeByFValue> open;

    NodePtr source_node = std::make_shared<Node>(source, std::vector<float>(2, 0),
                                                 heuristic_to_target(source));

    open.push(source_node);

    while (!open.empty()) {
        NodePtr cur_node = open.top();
        open.pop();
        num_generation += 1;

        if (local_dominance_check(cur_node) or global_dominance_check(cur_node, target)) {
            continue;
        }

        set_min_g2(cur_node);

        num_expansion += 1;

        if (cur_node->id != source) {
            if (frontiers[cur_node->id][cur_node->parent->id].empty()) {
                frontiers[cur_node->id][cur_node->parent->id] = cur_node->g;
            }
            else {
                frontiers[cur_node->id][cur_node->parent->id][1] = cur_node->g[1];
            }
        }

        if (global_stop_condition == true and cur_node->id == target) {
            last_solution = std::move(cur_node);
            continue;
        }

        const std::vector<Edge>& outgoing_edges = adj_matrix[cur_node->id];
        for (auto& outgoing_edge : outgoing_edges) {
            if (is_child_locally_dominated(cur_node, outgoing_edge)) {
                continue;
            }

            if (is_child_globally_dominated(cur_node, outgoing_edge, heuristic_to_target(outgoing_edge.target))) {
                continue;
            }

            NodePtr next_node = std::make_shared<Node>(
                outgoing_edge.target,
                std::vector<float>{cur_node->g[0] + outgoing_edge.cost[0], cur_node->g[1] + outgoing_edge.cost[1]},
                heuristic_to_target(outgoing_edge.target), cur_node, outgoing_edge.cost);
            open.push(next_node);
        }
    }

    std::vector<std::vector<std::vector<float>>> mvh_results(adj_matrix.size() + 1);
    for (size_t i = 0; i < adj_matrix.size() + 1; ++i) {
        mvh_results[i] = make_list_of_values(frontiers[i], heuristic_to_source(i));
    }

    runtime = static_cast<float>(std::clock() - start_time);

    std::ofstream PlotOutput("mvh.txt");

    for (int i = 1; i < adj_matrix.size() + 1; i++) {
        for (const auto& [key, value] : frontiers[i]) {
            PlotOutput << i << " " << key << " " << value[0] << " " << value[1] << std::endl;
        }
    }

    // Close the file
    PlotOutput.close();


    return mvh_results;
}
