//
// Created by crl on 27/07/2024.
//

#include "multivalued_heuristic/multi_objective_forward_search.h"

#include "solvers/apex.h"

#include <cassert>
#include <fstream>
#include <optional>
#include <boost/array.hpp>


void MultiObjectiveForwardSearch::update_closed_dr(const NodePtr& node_ptr) {
    std::vector<float> truncated_node = std::vector(node_ptr->g.begin() + 1, node_ptr->g.end());
    for (auto it = closed_dr[node_ptr->id].begin(); it != closed_dr[node_ptr->id].end();) {
        if (ApexSearch::is_strongly_dominated((*it), truncated_node)) {
            it = closed_dr[node_ptr->id].erase(it);
        }
        else {
            it++;
        }
    }
    closed_dr[node_ptr->id].insert(truncated_node);
}


void MultiObjectiveForwardSearch::update_closed(const NodePtr& node_ptr) {
    for (auto it = closed[node_ptr->id].begin(); it != closed[node_ptr->id].end();) {
        if (ApexSearch::is_strongly_dominated((*it)->g, node_ptr->g)) {
            it = closed[node_ptr->id].erase(it);
        }
        else {
            it++;
        }
    }
    closed[node_ptr->id].insert(node_ptr);
}

MultiObjectiveForwardSearch::MultiObjectiveForwardSearch(const AdjacencyMatrix& adj_matrix) : adj_matrix(adj_matrix) {
    closed.resize(adj_matrix.size() + 1);
    closed_dr.resize(adj_matrix.size() + 1);
}

bool MultiObjectiveForwardSearch::local_dominance_check(const NodePtr& node_ptr) const {
    if (closed[node_ptr->id].empty()) {
        return false;
    }
    auto last_closed_node = closed[node_ptr->id].end();
    last_closed_node--;

    if ((*last_closed_node)->g > node_ptr->g) {
        // if the last node in close is lexicographically greater than current
        // perfo rm dominance check without dr
        for (auto it = closed[node_ptr->id].begin(); it != closed[node_ptr->id].end(); it++) {
            if (ApexSearch::is_weakly_dominated(node_ptr->g, (*it)->g)) {
                return true;
            }
        }
        return false;
    }
    std::vector<float> truncated_value = std::vector(node_ptr->g.begin() + 1, node_ptr->g.end());
    for (auto it = closed_dr[node_ptr->id].begin(); it != closed_dr[node_ptr->id].end(); it++) {
        if (ApexSearch::is_weakly_dominated(truncated_value, (*it))) {
            return true;
        }
    }
    return false;
}

bool MultiObjectiveForwardSearch::global_dominance_check(const NodePtr& node_ptr, const size_t& target_id) {
    if (closed[target_id].empty()) {
        return false;
    }

    auto last_closed_node = closed[target_id].end();
    last_closed_node--;

    if ((*last_closed_node)->f > node_ptr->f) {
        // if the last node in close is lexicographically greater than current
        // perform dominance check without dr
        for (auto it = closed[target_id].begin(); it != closed[target_id].end(); it++) {
            if (ApexSearch::is_weakly_dominated(node_ptr->f, (*it)->f)) {
                return true;
            }
        }
        return false;
    }

    std::vector<float> truncated_node = std::vector(node_ptr->f.begin() + 1, node_ptr->f.end());
    for (const auto& other_node : closed_dr[target_id]) {
        if (ApexSearch::is_weakly_dominated(truncated_node, other_node)) {
            return true;
        }
    }
    return false;
}

std::optional<std::vector<float>> MultiObjectiveForwardSearch::get_first_undominated_heuristic_value(
    const std::vector<float>& g_value, const size_t& target, const std::vector<std::vector<float>>& node_mvh) {
    for (auto& heuristic : node_mvh) {
        std::vector<float> new_value(adj_matrix.num_of_objectives);
        for (int i = 0; i < new_value.size(); i++) {
            new_value[i] = g_value[i] + heuristic[i];
        }
        // bool flag = false;
        // for (const auto& it : closed[target]) {
        //     if (is_weakly_dominated(new_value, it->f)) {
        //         flag = true;
        //         break;
        //     }
        // }
        // if (flag == false) {
        //     return heuristic;
        // }
        std::vector<float> truncated_new_value =  std::vector(new_value.begin() + 1, new_value.end());
        bool flag = false;
        for (const auto & it : closed_dr[target]) {
            if (ApexSearch::is_weakly_dominated(truncated_new_value,  it)) {
                flag = true;
                break;
            }
        }
        if (flag == false) {
            return heuristic;
        }
    }
    return std::nullopt;
}

void MultiObjectiveForwardSearch::operator()(
    const size_t& source, const size_t& target,
    const MultiValuedHeuristic& heuristic, SolutionSet& solutions) {
    start_time = std::clock();
    std::priority_queue<NodePtr, std::vector<NodePtr>, CompareNodeByFValue> open;
    auto current_multi_valued_heuristic = heuristic[source];
    auto current_heuristic_value = current_multi_valued_heuristic[0];
    NodePtr source_node = std::make_shared<Node>(source, std::vector<float>(adj_matrix.num_of_objectives, 0),
                                                 current_heuristic_value, nullptr,
                                                 std::vector<float>(adj_matrix.num_of_objectives, 0));

    open.push(source_node);
    // float f1 = 0;

    while (!open.empty()) {
        auto node = open.top();
        open.pop();
        num_generation += 1;

        if (local_dominance_check(node)) {
            continue;
        }

        if (global_dominance_check(node, target)) {
            current_multi_valued_heuristic = heuristic[node->id];

            if (auto new_heuristic_value = get_first_undominated_heuristic_value(
                node->g, target, current_multi_valued_heuristic)) {
                NodePtr new_node = std::make_shared<Node>(
                    node->id, node->g, new_heuristic_value.value(), node->parent, node->c);

                open.push(new_node);
            }
            continue;
        }
        num_expansion += 1;

        closed[node->id].insert(node);
        update_closed_dr(node);

        const std::vector<Edge>& outgoing_edges = adj_matrix[node->id];

        for (const auto& outgoing_edge : outgoing_edges) {
            std::vector new_g(node->g);
            for (int i = 0; i < new_g.size(); i++) {
                new_g[i] += outgoing_edge.cost[i];
            }
            current_multi_valued_heuristic = heuristic[outgoing_edge.target];

            auto new_h =
                get_first_undominated_heuristic_value(new_g, target, current_multi_valued_heuristic);

            if (new_h == std::nullopt) {
                continue;
            }

            NodePtr successor_node = std::make_shared<Node>(
                outgoing_edge.target, new_g, new_h.value(), node, outgoing_edge.cost);

            if (local_dominance_check(successor_node)) {
                continue;
            }
            open.push(successor_node);
        }
    }

    for (const auto& solution : closed[target]) {
        solutions.push_back(solution);
    }

    runtime = std::clock() - start_time;
}
