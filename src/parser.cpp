//
// Created by crl on 13/07/2024.
//

#include "parser.h"

#include <boost/algorithm/string.hpp>
#include <fstream>
#include <iostream>

AdjacencyMatrix Parser::parse_graph(const std::string &default_files_directory) {
   std::string distances_filename = default_files_directory + "\\distances.gr";
   std::string times_filename = default_files_directory + "\\times.gr";
  std::ifstream distances_infile(distances_filename);
  if (!distances_infile) {
    std::cerr << "Error opening file: " << distances_filename << std::endl;
    exit(1);
  }
  std::ifstream times_infile(times_filename);
  if (!times_infile) {
    std::cerr << "Error opening file: " << times_filename << std::endl;
    exit(1);
  }

  std::vector<Edge> edges;
  size_t max_node_num = 0;
  std::string distances_line, times_line;
  while ((!distances_infile.eof()) && (!times_infile.eof())) {
    std::getline(distances_infile, distances_line);
    std::getline(times_infile, times_line);

    if ((distances_line.empty()) || (times_line.empty())) {
      break;
    }

    std::vector<std::string> decomposed_distances_line, decomopsed_times_line;

    split(decomposed_distances_line, distances_line, boost::is_any_of(" "));
    split(decomopsed_times_line, times_line, boost::is_any_of(" "));

    std::string type = decomposed_distances_line[0];
    if ((std::strcmp(type.c_str(), "c") == 0) ||
        (std::strcmp(type.c_str(), "p") == 0)) {
      continue; // comment or problem lines, not part of the graph
    }

    if ((decomposed_distances_line[0] != decomopsed_times_line[0]) ||
        (decomposed_distances_line[1] != decomopsed_times_line[1]) ||
        (decomposed_distances_line[2] != decomopsed_times_line[2])) {
      // arc_sign src dest should be same in both files
      std::cerr << "Missmatching input files " << distances_filename
                << times_filename << std::endl;
      exit(1);
    }

    if (std::strcmp(type.c_str(), "a") == 0) { // arc
      Edge e(std::stoul(decomposed_distances_line[1]),
             std::stoul(decomposed_distances_line[2]),
             {static_cast<float>(std::stoul(decomposed_distances_line[3])),
              static_cast<float>(std::stoul(decomopsed_times_line[3]))});
      edges.push_back(e);
      max_node_num = std::max({max_node_num, e.source, e.target});
    }
  }
  return {max_node_num, edges, true};
}

