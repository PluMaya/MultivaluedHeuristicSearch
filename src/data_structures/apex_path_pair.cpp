//
// Created by crl on 13/07/2024.
//

#include "data_structures/apex_path_pair.h"

bool is_bounded(const NodePtr &apex, const NodePtr &node, const EPS &eps) {
  for (int i = 0; i < apex->f.size(); i++) {
    if (node->f[i] > (1 + eps[i]) * apex->f[i]) {
      return false;
    }
  }
  return true;
}

ApexPathPair::ApexPathPair(const ApexPathPairPtr &parent, const Edge &edge,
                           const std::vector<float> &h)
    : id(edge.target), h(h) {
  std::vector<float> new_apex_g(parent->apex->g);
  std::vector<float> new_g(parent->path_node->g);

  for (int i = 0; i < new_apex_g.size(); i++) {
    new_apex_g[i] += edge.cost[i];
    new_g[i] += edge.cost[i];
  }
  apex = std::make_shared<Node>(id, new_apex_g, h);
  path_node = std::make_shared<Node>(id, new_g, h);
}

bool ApexPathPair::update_nodes_by_merge_if_bounded(
    const ApexPathPairPtr &other, const std::vector<double> &eps) {
  // Returns true on sucessful merge
  if (id != other->id) {
    return false;
  }

  std::vector f_value = {std::min(apex->f[0], other->apex->f[0]), std::min(apex->f[1], other->apex->f[1])};

  // apex is not bunded
  if (path_node->f[0] > (1 + eps[0]) * f_value[0] or path_node->f[1] > (1 + eps[1]) * f_value[1]) {
    return false;
  }

  this->apex->g = {std::min(apex->g[0], other->apex->g[0]), std::min(apex->g[1], other->apex->g[1])};
  this->apex->f = {std::min(apex->f[0], other->apex->f[0]), std::min(apex->f[1], other->apex->f[1])};
  return true;
}

bool ApexPathPair::update_apex_by_merge_if_bounded(
    const NodePtr &other_apex, const std::vector<double> &eps) {
  std::vector<float> f = {std::min(apex->f[0], other_apex->f[0]),
            std::min(apex->f[1], other_apex->f[1])};
  if (path_node->f[0] > (1 + eps[0]) * f[0] or path_node->f[1] > (1 + eps[1]) * f[1]) {
    return false;
  }

  apex->f = f;
  apex->g = {std::min(apex->g[0], other_apex->g[0]),
            std::min(apex->g[1], other_apex->g[1])};
  return true;
}

