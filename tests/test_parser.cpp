//
// Created by crl on 17/07/2024.
//

#include "data_structures/adjacency_matrix.h"
#include "parser.h"

#include <sstream>

void test_parser_doesnt_crash() {
  // Simulate input file content
  std::string coordinates_filename =
      "C:/Users/crl/CLionProjects/MultivaluedHeuristicSearch/resources/NYC/"
      "USA-road-d.NY.co";
  std::string distances_filename =
      "C:/Users/crl/CLionProjects/MultivaluedHeuristicSearch/resources/NYC/"
      "USA-road-d.NY.gr";
  std::string times_filename =
      "C:/Users/crl/CLionProjects/MultivaluedHeuristicSearch/resources/NYC/"
      "USA-road-t.NY.gr";
  AdjacencyMatrix adjecency_matrix = Parser::parse_graph(
      coordinates_filename, distances_filename, times_filename);
}

// int main() {
//   test_parser_doesnt_crash();
//   return 0;
// }
