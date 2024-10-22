//
// Created by crl on 17/07/2024.
//

#include "data_structures/adjacency_matrix.h"
#include "doctest.h"
#include "parser.h"

#include <sstream>

TEST_CASE("test parser doesnt crash") {
  // Simulate input file content
  const auto parser = Parser();
  AdjacencyMatrix adjecency_matrix = parser.default_graph();
  CHECK(adjecency_matrix.size() != 0);
}
