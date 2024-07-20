// #include <iostream>
// #include <fstream>
//
// #include <boost/program_options.hpp>
//
// #include "data_structures/edge.h"
// #include "parser.h"
//
// using namespace std;
//
// const std::string resource_path = "resources/";
// const std::string output_path = "output/";
//
//
// int main(int argc, char** argv)
// {
//     std::vector<string> objective_files;
//
//     boost::program_options::options_description options_description("Allowed options");
//     options_description.add_options()
//         ("help,h", "produce help message")
//         ("start,s", boost::program_options::value<int>()->default_value(-1), "start location")
//         ("goal,g", boost::program_options::value<int>()->default_value(-1), "goal location")
//         ("query,q", boost::program_options::value<std::string>()->default_value(""), "number of agents")
//         ("map,m", boost::program_options::value<std::vector<string>>(&objective_files)->multitoken(),
//          "files for edge weight")
//         ("eps,e", boost::program_options::value<double>()->default_value(0), "approximation factor")
//         ("algorithm,a", boost::program_options::value<std::string>()->default_value("Apex"),
//          "solvers (BOA or Apex)")
//         ("cutoffTime,t", boost::program_options::value<int>()->default_value(300), "cutoff time (seconds)")
//         ("output,o", boost::program_options::value<std::string>()->required(), "Name of the output file")
//         ("logging_file", boost::program_options::value<std::string>()->default_value(""), "logging file");
//
//     boost::program_options::variables_map variables_map;
//     store(parse_command_line(argc, argv, options_description), variables_map);
//
//     if (variables_map.count("help"))
//     {
//         std::cout << options_description << std::endl;
//         return 1;
//     }
//
//     notify(variables_map);
//
//     if (!variables_map["query"].as<std::string>().empty())
//     {
//         if (variables_map["start"].as<int>() != -1 || variables_map["goal"].as<int>() != -1)
//         {
//             std::cerr << "query file and start/goal cannot be given at the same time !" << std::endl;
//             return -1;
//         }
//     }
//
//     // Load files
//     size_t graph_size;
//     std::vector<Edge> edges;
//
//     for (const auto& file : objective_files)
//     {
//         std::cout << file << std::endl;
//     }
//
//
//     if (load_files(objective_files, edges, graph_size) == false)
//     {
//         // std::cout << "Failed to load gr files" << std::endl;
//         return -1;
//     }
//
//     std::cout << "Graph Size: " << graph_size << std::endl;
//
//     if (!variables_map["query"].as<std::string>().empty())
//     {
//         run_query(graph_size, edges, variables_map["query"].as<std::string>(),
//                   variables_map["output"].as<std::string>(),
//                   variables_map["algorithm"].as<std::string>(), ms, logger, variables_map["eps"].as<double>(),
//                   variables_map["cutoffTime"].as<int>());
//     }
//     else
//     {
//         single_run_map(graph_size, edges, variables_map["start"].as<int>(), variables_map["goal"].as<int>(),
//                        variables_map["output"].as<std::string>(),
//                        variables_map["algorithm"].as<std::string>(), ms, logger, variables_map["eps"].as<double>(),
//                        variables_map["cutoffTime"].as<int>());
//     }
//
//     return 0;
// }
