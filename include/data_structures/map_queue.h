//
// Created by crl on 15/07/2024.
//

#ifndef MAP_QUEUE_H
#define MAP_QUEUE_H
#include "apex_path_pair.h"

#include <list>
#include <queue>
#include <vector>

class MapQueue {
public:
  std::priority_queue<ApexPathPairPtr, std::vector<ApexPathPairPtr>,
                      CompareApexPathPairByValue> heap;
  std::vector<std::list<ApexPathPairPtr>> open_map;

  explicit MapQueue(size_t graph_size)
      : open_map(graph_size, std::list<ApexPathPairPtr>()){};
  bool empty() const { return heap.empty(); }
  ApexPathPairPtr pop();
  void insert(const ApexPathPairPtr &pp);
  std::list<ApexPathPairPtr> &get_open(size_t id) { return open_map[id]; };
};

#endif // MAP_QUEUE_H
