//
// Created by crl on 13/07/2024.
//

#ifndef PARSER_H
#define PARSER_H

#include <data_structures/adjacency_matrix.h>

class Parser {
public:
  std::string default_coordinates_filename =
      "C:/Users/crl/CLionProjects/MultivaluedHeuristicSearch/resources/NYC/"
      "USA-road-d.NY.co";
  std::string default_distances_filename =
      "C:/Users/crl/CLionProjects/MultivaluedHeuristicSearch/resources/NYC/"
      "USA-road-d.NY.gr";
  std::string default_times_filename =
      "C:/Users/crl/CLionProjects/MultivaluedHeuristicSearch/resources/NYC/"
      "USA-road-t.NY.gr";
  Parser() = default;
  static AdjacencyMatrix parse_graph(const std::string &coordinates_file,
                                     const std::string &distances_filename,
                                     const std::string &times_filename);

  AdjacencyMatrix default_graph() const {
    return parse_graph(default_coordinates_filename, default_distances_filename,
                       default_times_filename);
  }
};

#endif // PARSER_H
