//
// Created by crl on 13/07/2024.
//

#ifndef APEX_PATH_PAIR_H
#define APEX_PATH_PAIR_H
#include "definitions.h"
#include "edge.h"
#include "node.h"

bool is_bounded(const NodePtr& apex, const NodePtr& node, const EPS& eps);

class ApexPathPair;
using ApexPathPairPtr = std::shared_ptr<ApexPathPair>;

class ApexPathPair {
public:
    size_t id;
    NodePtr apex;
    NodePtr path_node;
    NodePtr parent;
    bool is_active = true;

    const std::vector<float> h;

    ApexPathPair(const NodePtr& apex, const NodePtr& path_node,
                 const std::vector<float>& h)
        : id(apex->id), apex(apex), path_node(path_node),
          parent(path_node->parent), h(h) {
    };

    ApexPathPair(const ApexPathPairPtr& parent, const Edge& edge, const std::vector<float>& h);

    bool update_nodes_by_merge_if_bounded(const ApexPathPairPtr& other,
                                          const EPS& eps);

    bool update_apex_by_merge_if_bounded(const NodePtr& other_apex,
                                         const EPS& eps);
};

struct CompareApexPathPairByValue {
    bool operator()(const ApexPathPairPtr& a, const ApexPathPairPtr& b) const {
        return std::lexicographical_compare(b->apex->f.begin(), b->apex->f.end(),
                                            a->apex->f.begin(), a->apex->f.end());
    }
};

#endif // APEX_PATH_PAIR_H
