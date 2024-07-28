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

ApexPathPair::ApexPathPair(const ApexPathPairPtr &parent, const Edge &edge, const std::vector<float>& h)
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

  NodePtr new_apex =
      std::make_shared<Node>(this->apex->id, this->apex->g, this->apex->h);

  // Merge apex
  for (int i = 0; i < other->apex->g.size(); i++) {
    if (other->apex->g[i] < new_apex->g[i]) {
      new_apex->g[i] = other->apex->g[i];
      new_apex->f[i] = other->apex->f[i];
    }
  }

  if (!is_bounded(new_apex, path_node, eps)) {
    return false;
  }

  this->apex = new_apex;
  return true;
}

bool ApexPathPair::update_apex_by_merge_if_bounded(
    const NodePtr &other_apex, const std::vector<double> &eps) {
  NodePtr new_apex =
      std::make_shared<Node>(this->apex->id, this->apex->g, this->apex->h);
  bool update_flag = false;

  // Merge apex
  for (int i = 0; i < other_apex->g.size(); i++) {
    if (other_apex->f[i] < new_apex->f[i]) {
      new_apex->g[i] = other_apex->g[i];
      new_apex->f[i] = other_apex->f[i];
      if (path_node->f[i] > (1 + eps[i]) * new_apex->f[i]) {
        return false;
      }

      update_flag = true;
    }
  }
  if (update_flag) {
    apex = new_apex;
  }
  return true;
}
