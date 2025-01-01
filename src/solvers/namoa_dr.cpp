#include <fstream>
#include <iostream>
#include <list>
#include <queue>
#include <solvers/namoa_dr.h>

NAMOAdr::NAMOAdr(const AdjacencyMatrix& adj_matrix)
    : AbstractSolver(adj_matrix, {}) {
}

bool is_dominated_dr(const NodePtr& node, const NodePtr& other_node) {
    // returns true if node is weakly dominated by other_node
    for (int i = 1; i < node->f.size(); i++) {
        if (node->f[i] < other_node->f[i]) {  // it is enough to have one entry that is lower than the other node
            return false;
        }
    }
    return true;
}


bool is_dominated_dr(const NodePtr& node, const std::list<NodePtr>& list) {
    for (const auto& n : list) {
        if (is_dominated_dr(node, n)) {
            return true;
        }
    }
    return false;
}


void add_node_dr(const NodePtr& node, std::list<NodePtr>& list) {
    for (auto it = list.begin(); it != list.end();) {
        if (is_dominated_dr((*it), node)) {
            it = list.erase(it);
        }
        else {
            it++;
        }
    }
    list.push_back(node);
}


void NAMOAdr::operator()(const size_t& source, const size_t& target,
                         const Heuristic& heuristic, SolutionSet& solutions,
                         const unsigned int time_limit) {
    init_search();
    std::priority_queue<NodePtr, std::vector<NodePtr>, CompareNodeByFValue> open;
    std::vector<std::list<NodePtr>> closed(this->adj_matrix.size() + 1);
    // Init open heap
    auto node = std::make_shared<Node>(source, std::vector<float>(adj_matrix.num_of_objectives, 0.0f),
                                       heuristic(source));
    open.push(node);

    while (!open.empty()) {
        // Pop min from queue and process
        node = open.top();
        open.pop();
        num_generation += 1;

        // Dominance check
        if (is_dominated_dr(node, closed[node->id]) || is_dominated_dr(node, closed[target])) {
            continue;
        }

        add_node_dr(node, closed[node->id]);
        num_expansion += 1;

        if (node->id == target) {
            solutions.push_back(node);
            continue;
        }

        // Check to which neighbors we should extend the paths
        const std::vector<Edge>& outgoing_edges = adj_matrix[node->id];

        for (const auto& outgoing_edge : outgoing_edges) {
            size_t next_id = outgoing_edge.target;
            std::vector<float> next_g(node->g.size());
            for (size_t i = 0; i < next_g.size(); i++) {
                next_g[i] = node->g[i] + outgoing_edge.cost[i];
            }
            auto next_h = heuristic(next_id);
            auto next = std::make_shared<Node>(next_id, next_g, next_h, node);

            if (is_dominated_dr(next, closed[next_id]) || is_dominated_dr(next, closed[target])) {
                continue;
            }
            open.push(next);
        }
    }

    runtime = static_cast<float>(std::clock() - start_time);
}
