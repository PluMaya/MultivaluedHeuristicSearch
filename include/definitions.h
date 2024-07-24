//
// Created by crl on 13/07/2024.
//

#ifndef DEFINITIONS_H
#define DEFINITIONS_H
#include <array>
#include <functional>
#include <limits>
#include <ostream>
#include <vector>

constexpr size_t MAX_COST = std::numeric_limits<size_t>::max();

template <typename T> using Pair = std::array<T, 2>;

template <typename T>
std::ostream &operator<<(std::ostream &stream, const Pair<T> pair) {
  stream << "[" << pair[0] << ", " << pair[1] << "]";
  return stream;
}

using Heuristic = std::function<std::vector<float>(size_t)>;
using MultiValuedHeuristic =
    std::function<std::vector<std::vector<float>>(size_t)>;

using EPS = std::vector<double>;

#endif // DEFINITIONS_H
