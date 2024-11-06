//
// Created by crl on 13/07/2024.
//

#ifndef NODE_H
#define NODE_H
#include <memory>
#include <utility>
#include <vector>

/**
 * Class representing a node in a search problem
 */
class Node;
using NodePtr = std::shared_ptr<Node>;

class Node {
public:
  size_t id;
  std::vector<float> g;
  std::vector<float> h;
  std::vector<float> f;
  const NodePtr& parent;
  const std::vector<float>& c;

  /**
   * Constructing a new node.
   *
   * @param id the ID (state?) of a node
   * @param g the g-value of a node
   * @param h the h-value (heuristic) of a node
   * @param parent the parent of a node
   */
  Node(const size_t id, const std::vector<float> &g,
       const std::vector<float> &h, const NodePtr& parent = nullptr, const std::vector<float> &c = std::vector<float>(2, 0 ))
      : id(id), g(g), h(h), f(g.size()), parent(parent), c(c){
    for (int i = 0; i < g.size(); i++) {
      f[i] = g[i] + h[i];
    }
  };
};

struct CompareNodeByFValue {
  bool operator()(const NodePtr& a, const NodePtr& b) const {

    return std::lexicographical_compare(b->f.begin(), b->f.end(),
                                        a->f.begin(), a->f.end());
  }
};

#endif // NODE_H
