//
// Created by crl on 13/07/2024.
//

#ifndef PARSER_H
#define PARSER_H

#include <data_structures/adjacency_matrix.h>

#include <utility>

inline std::string DEFAULT_FILES_DIRECTORY = "C:/Users/crl/CLionProjects/MultivaluedHeuristicSearch/resources/NYC";

class Parser {
public:
  std::string files_directory;

  explicit Parser(std::string files_directory)
      : files_directory(std::move(files_directory)) {}
  explicit Parser()
      : files_directory(DEFAULT_FILES_DIRECTORY) {}

  static AdjacencyMatrix parse_graph(const std::string &default_files_directory);

  [[nodiscard]] AdjacencyMatrix default_graph() const {
    return parse_graph(files_directory);
  }
};

#endif // PARSER_H
