//
// Created by crl on 14/07/2024.
//

#include <iostream>
#include <queue>
#include <solvers/boa.h>

BOAStar::BOAStar(const AdjacencyMatrix& adj_matrix)
    : AbstractSolver(adj_matrix, {0, 0}) {
}

void BOAStar::operator()(const size_t& source, const size_t& target,
                         const Heuristic& heuristic, SolutionSet& solutions,
                         const unsigned int time_limit) {
    init_search();
    start_time = std::clock();
    std::vector<NodePtr> closed;
    // Vector to hold mininum cost of 2nd criteria per node
    std::vector<float> min_g2(adj_matrix.size() + 1,
                              static_cast<float>(MAX_COST));

    // Init open heap
    std::priority_queue<NodePtr, std::vector<NodePtr>, CompareNodeByFValue> open;

    auto node = std::make_shared<Node>(source, std::vector<float>(2, 0.0f),
                                       heuristic(source));
    open.push(node);

    while (!open.empty()) {
        if ((std::clock() - start_time) / CLOCKS_PER_SEC > time_limit) {
            return;
        }

        // Pop min from queue and process
        node = open.top();
        open.pop();
        num_generation += 1;

        // Dominance check
        if ((node->f[1]) >= min_g2[target] ||
            (node->g[1] >= min_g2[node->id])) {
            closed.push_back(node);
            continue;
        }

        min_g2[node->id] = node->g[1];
        num_expansion += 1;

        if (node->id == target) {
            solutions.push_back(node);
            continue;
        }

        // Check to which neighbors we should extend the paths
        const std::vector<Edge>& outgoing_edges = adj_matrix[node->id];

        for (const auto& outgoing_edge : outgoing_edges) {
            size_t next_id = outgoing_edge.target;
            std::vector<float> next_g = {
                node->g[0] + outgoing_edge.cost[0],
                node->g[1] + outgoing_edge.cost[1]
            };
            auto next_h = heuristic(next_id);

            // Dominance check
            if ((next_g[1] + next_h[1]) >= min_g2[target] ||
                (next_g[1] >= min_g2[next_id])) {
                continue;
            }

            auto next = std::make_shared<Node>(next_id, next_g, next_h, node);

            open.push(next);
        }
        closed.push_back(node);
    }

    for (auto solution : solutions) {
        std::cout << solution->g[0] << " " << solution->g[1] << std::endl;
    }

    runtime = static_cast<float>(std::clock() - start_time);
}
