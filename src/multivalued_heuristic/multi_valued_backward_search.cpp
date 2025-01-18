//
// Created by crl on 22/07/2024.
//

#include <fstream>
#include <iostream>
#include <multivalued_heuristic/multi_objective_backward_search.h>


void MultiObjectiveBackwardSearch::update_close_dr(const ApexPathPairPtr& ap) {
    std::vector<float> truncated_node = std::vector(ap->apex->g.begin() + 1, ap->apex->g.end());
    for (auto it = closed_dr[ap->id].begin(); it != closed_dr[ap->id].end();) {
        if (ApexSearch::is_strongly_dominated((*it), truncated_node)) {
            it = closed_dr[ap->id].erase(it);
        }
        else {
            it++;
        }
    }
    closed_dr[ap->id].insert(truncated_node);
}

bool MultiObjectiveBackwardSearch::local_dominance_check(const ApexPathPairPtr& ap) const {
    if (closed_dr[ap->id].empty()) {
        return false;
    };
    std::vector<float> truncated_vector = std::vector(ap->apex->g.begin() + 1, ap->apex->g.end());
    for (auto it = closed_dr[ap->id].begin(); it != closed_dr[ap->id].end(); ++it) {
        if (ApexSearch::is_weakly_dominated(truncated_vector, (*it))) {
            return true;
        }
    }
    return false;
}

MultiObjectiveBackwardSearch::MultiObjectiveBackwardSearch(const AdjacencyMatrix& adj_matrix, EPS eps) :
    adj_matrix(adj_matrix), eps(std::move(eps)) {
    closed_dr.resize(adj_matrix.size() + 1);
}

std::vector<std::vector<float>>
MultiObjectiveBackwardSearch::make_list_of_values(const ApexSolutionSet& apex_solution_set,
                                                  const std::vector<float>& heuristic_value) const {
    std::vector<std::vector<float>> result = {};
    if (apex_solution_set.empty()) {
        return result;
    }
    
    // Perform stair operations on the result
    // std::vector min_objective_values(adj_matrix.num_of_objectives, std::numeric_limits<float>::max());
    // for (const auto& apex_solution : apex_solution_set) {
    //     for (size_t i = 0; i < min_objective_values.size(); ++i) {
    //         min_objective_values[i] = std::min(min_objective_values[i],
    //                                            apex_solution->apex->f[i] - apex_solution->apex->h[i]);
    //     }
    // }
    if (adj_matrix.num_of_objectives == 2) {
        for (int k = 0; k < apex_solution_set.size() - 1; k++) {
            std::vector<float> fixed_solution = std::vector<float>(adj_matrix.num_of_objectives, 0);
            fixed_solution[0] = apex_solution_set[k]->apex->f[0] - apex_solution_set[k]->apex->h[0];
            fixed_solution[1] = apex_solution_set[k + 1]->apex->f[1] - apex_solution_set[k + 1]->apex->h[1];

            // for (int i = 0; i < adj_matrix.num_of_objectives; i++) {
            //     if (fixed_solution[i] == min_objective_values[i]) {
            //         fixed_solution[i] = heuristic_value[i];
            //     }
            // }
            result.push_back(fixed_solution);
        }

    }
    for (const auto& apex_solution : apex_solution_set) {
        std::vector<float> fixed_solution = std::vector<float>(adj_matrix.num_of_objectives, 0);
        for (int i = 0; i < adj_matrix.num_of_objectives; i++) {
            fixed_solution[i] = apex_solution->apex->f[i] - apex_solution->apex->h[i];
        }
        // for (int i = 0; i < adj_matrix.num_of_objectives; i++) {
        //     if (fixed_solution[i] == min_objective_values[i]) {
        //         fixed_solution[i] = heuristic_value[i];
        //     }
        // }
        result.push_back(fixed_solution);

    }
    return result;
}


void insert(ApexPathPairPtr& ap, MapQueue& queue, const EPS& eps) {
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


using BackwardSearchSolutionSet = std::unordered_map<size_t, ApexSolutionSet>;

MultiValuedHeuristic
MultiObjectiveBackwardSearch::operator()(const size_t& source, const size_t& target,
                                         const Heuristic& heuristic_to_target,
                                         const Heuristic& heuristic_to_source,
                                         const std::vector<float>& upper_bound) {
    start_time = std::clock();

    BackwardSearchSolutionSet frontiers;

    MapQueue open(adj_matrix.size() + 1);

    NodePtr source_node = std::make_shared<Node>(source, std::vector<float>(adj_matrix.num_of_objectives, 0),
                                                 heuristic_to_target(source));
    ApexPathPairPtr ap = std::make_shared<ApexPathPair>(
        source_node, source_node, heuristic_to_target(target));
    open.insert(ap);
    std::vector<ApexPathPairPtr> solutions;

    while (!open.empty()) {
        ap = open.pop();
        num_generation += 1;

        if (ap->is_active == false) {
            continue;
        }
        if (local_dominance_check(ap) or !ApexSearch::is_weakly_dominated(upper_bound, ap->apex->g)) {
            continue;
        }

        update_close_dr(ap);
        frontiers[ap->id].push_back(ap);

        num_expansion += 1;

        const std::vector<Edge>& outgoing_edges = adj_matrix[ap->id];
        for (const auto& outgoing_edge : outgoing_edges) {
            ApexPathPairPtr next_ap = std::make_shared<ApexPathPair>(
                ap, outgoing_edge, heuristic_to_target(outgoing_edge.target));
            if (local_dominance_check(next_ap) or !ApexSearch::is_weakly_dominated(upper_bound, next_ap->apex->g)) {
                continue;
            }
            insert(next_ap, open, eps);
        }
    }

    std::vector<std::vector<std::vector<float>>> mvh_results(adj_matrix.size() + 1);
    for (size_t i = 0; i < adj_matrix.size() + 1; ++i) {
        mvh_results[i] = make_list_of_values(frontiers[i], heuristic_to_source(i));
    }

    runtime = static_cast<float>(std::clock() - start_time);


    std::ofstream PlotOutput("mvh.txt");

    for (int i = 1; i < adj_matrix.size() + 1; i++) {
        for (const auto& it : mvh_results[i]) {
            PlotOutput << i << " ";
            for (const auto& it2 : it) {
                PlotOutput << it2 << " ";
            }
            PlotOutput << std::endl;
        }
    }
    // Close the file
    PlotOutput.close();



    return mvh_results;
}
