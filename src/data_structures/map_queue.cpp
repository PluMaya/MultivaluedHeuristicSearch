//
// Created by crl on 15/07/2024.
//

#include "data_structures/map_queue.h"

#include <algorithm>

ApexPathPairPtr MapQueue::pop() {
  ApexPathPairPtr pp = heap.top();
  heap.pop();
  // TODO: something is fishy here with the iterator
  std::list<ApexPathPairPtr> &relevant_pps = open_map[pp->id];
  for (auto iter = relevant_pps.begin(); iter != relevant_pps.end(); ++iter) {
    if (pp == *iter) {
      relevant_pps.erase(iter);
      break;
    }
  }
  return pp;
}

void MapQueue::insert(const ApexPathPairPtr &pp) {
  heap.push(pp);
  open_map[pp->id].push_back(pp);
}
