//
// Created by crl on 22/07/2024.
//

#include <iostream>
#include <multivalued_heuristic/backward_search.h>

#include <utility>

void BackwardSearch::insert(ApexPathPairPtr& ap, MapQueue& queue) {
    std::list<ApexPathPairPtr>& relevant_aps = queue.get_open(ap->id);
    for (auto relevant_ap = relevant_aps.begin();
         relevant_ap != relevant_aps.end(); ++relevant_ap) {
        if ((*relevant_ap)->is_active == false) {
            relevant_aps.erase(relevant_ap);
            continue;
        }
        if (ap->update_nodes_by_merge_if_bounded(*relevant_ap, eps)) {
            if ((ap->apex != (*relevant_ap)->apex) || (ap->path_node != (*relevant_ap)->path_node)) {
                (*relevant_ap)->is_active = false;
                relevant_aps.erase(relevant_ap);
                queue.insert(ap);
            }
            return;
        }
    }
    queue.insert(ap);
}

bool BackwardSearch::global_dominance_check(const ApexPathPairPtr& ap) const {
    if (global_stop_condition == false) {
        return false;
    }
    if (last_solution == nullptr) {
        return false;
    }
    if (is_bounded(ap->apex, last_solution->path_node, eps)) {
        last_solution->update_apex_by_merge_if_bounded(ap->apex, eps);
        return true;
    }
    return false;
}

BackwardSearch::BackwardSearch(const AdjacencyMatrix& adj_matrix, EPS eps)
    : adj_matrix(adj_matrix), eps(std::move(eps)) {
    expanded.resize(adj_matrix.size() + 1);
    min_g2.resize(adj_matrix.size() + 1, static_cast<float>(MAX_COST));
}

bool BackwardSearch::is_dominated(const ApexPathPairPtr& ap) const {
    if (local_dominance_check(ap)) {
        return true;
    }
    return global_dominance_check(ap);
}

bool BackwardSearch::is_child_locally_dominated(const ApexPathPairPtr& parent, const Edge& edge) {
    return parent->apex->g[1] + edge.cost[1] >= min_g2[edge.target];
}

bool BackwardSearch::is_child_globally_dominated(const ApexPathPairPtr& parent, const Edge& edge,
                                                 const std::vector<float>& h) {
    if (global_stop_condition == false) {
        return false;
    }
    if (last_solution == nullptr) {
        return false;
    }

    std::vector f_value = {
        parent->apex->g[0] + edge.cost[0] + h[0],
        parent->apex->g[1] + edge.cost[1] + h[1]
    };

    // apex is not bunded
    if (last_solution->path_node->f[0] > (1 + eps[0]) * f_value[0] or
        last_solution->path_node->f[1] > (1 + eps[1]) * f_value[1]) {
        return false;
    }

    // update the last solution since it is bounded
    last_solution->apex->f = {
        std::min(last_solution->apex->f[0], f_value[0]),
        std::min(last_solution->apex->f[1], f_value[1])
    };
    return true;
}

std::vector<std::vector<float>>
BackwardSearch::make_list_of_values(const ApexSolutionSet& apex_solution_set,
                    const std::vector<float>& heuristic_value) {
    std::vector<std::vector<float>> result = {};
    if (apex_solution_set.empty() || apex_solution_set.size() == 1) {
        result.push_back(heuristic_value);
    }
    else {
        // Perform satir operations on the result
        for (int i = 0; i < apex_solution_set.size() - 1; i++) {
            std::vector<float> solution_value = {
                apex_solution_set[i]->apex->f[0] - apex_solution_set[i]->apex->h[0],
                apex_solution_set[i + 1]->apex->f[1] - apex_solution_set[i + 1]->apex->h[1]
            };
            result.push_back(solution_value);
        }
        if (global_stop_condition == true) {
            result.at(0).at(0) = heuristic_value.at(0);
            result.at(result.size() - 1).at(1) = heuristic_value.at(1);
        }
    }
    return result;
}

MultiValuedHeuristic
BackwardSearch::operator()(const size_t& source, const size_t& target,
                           const Heuristic& heuristic_to_target,
                           const Heuristic& heuristic_to_source,
                           bool global_stop_condition) {
    start_time = std::clock();
    this->global_stop_condition = global_stop_condition;

    BackwardSearchSolutionSet frontiers;

    MapQueue open(adj_matrix.size() + 1);

    NodePtr source_node = std::make_shared<Node>(source, std::vector<float>(2, 0),
                                                 heuristic_to_target(source));
    ApexPathPairPtr ap = std::make_shared<ApexPathPair>(
        source_node, source_node, heuristic_to_target(target));
    open.insert(ap);

    while (!open.empty()) {
        ap = open.pop();
        num_generation += 1;

        if (ap->is_active == false) {
            continue;
        }
        if (is_dominated(ap)) {
            continue;
        }

        set_min_g2(ap);

        num_expansion += 1;

        if (frontiers[ap->id].empty()) {
            frontiers[ap->id].push_back(ap);
        } else {
            auto back_apex = frontiers[ap->id].back()->apex;
            if (ap->path_node->f[0] > (1 + eps[0]) * back_apex->f[0] or ap->path_node->f[1] > (1 + eps[1]) * back_apex->f[1]) {
                frontiers[ap->id].push_back(ap);
            }
        }

        if (global_stop_condition == true and ap->id == target) {
            last_solution = move(ap);
            continue;
        }

        const std::vector<Edge>& outgoing_edges = adj_matrix[ap->id];
        for (const auto& outgoing_edge : outgoing_edges) {
            if (is_child_locally_dominated(ap, outgoing_edge)) {
                continue;
            }

            if (is_child_globally_dominated(ap, outgoing_edge, heuristic_to_target(outgoing_edge.target))) {
                continue;
            }

            ApexPathPairPtr next_ap = std::make_shared<ApexPathPair>(
                ap, outgoing_edge, heuristic_to_target(outgoing_edge.target));

            insert(next_ap, open);
        }
    }

    // for (auto solution : frontiers[target]) {
    //     std::vector solution_value = {
    //         solution->apex->f[0] - solution->apex->h[0], solution->apex->f[1] - solution->apex->h[1]
    //     };
    //     std::vector node_vale = {
    //         solution->path_node->f[0] - solution->path_node->h[0], solution->path_node->f[1] - solution->path_node->h[1]
    //     };
    //     std::cout << solution_value[0] << " " << solution_value[1] << "\t" <<
    //         node_vale[0] << " " << node_vale[1] << std::endl;
    // }

    std::vector<std::vector<std::vector<float>>> mvh_results(adj_matrix.size() + 1);
    for (size_t i = 0; i < adj_matrix.size() + 1; ++i) {
        mvh_results[i] = make_list_of_values(frontiers[i], heuristic_to_source(i));
    }

    runtime = static_cast<float>(std::clock() - start_time);

    return mvh_results;
}
