#include "experiment_utils.h"
#include "parser.h"

#include <fstream>
#include <iostream>

#include <boost/program_options.hpp>

using namespace std;

const std::string output_path = "../output/";

int main(int argc, char **argv) {
  boost::program_options::options_description options_description(
      "Allowed options");
  options_description.add_options()("help,h", "produce help message")(
      "start,s", boost::program_options::value<int>()->default_value(-1),
      "start location")("goal,g",
                        boost::program_options::value<int>()->default_value(-1),
                        "goal location")(
      "query,q",
      boost::program_options::value<std::string>()->default_value(""),
      "number of agents")(
      "map,m", boost::program_options::value<std::string>()->default_value(""),
      "directory for edge weights files")(
      "e1,e1", boost::program_options::value<double>()->default_value(0),
      "approximation factor obj1")(
      "e2, e2", boost::program_options::value<double>()->default_value(0),
      "approximation factor obj2")(
      "algorithm,a",
      boost::program_options::value<std::string>()->default_value("Apex"),
      "solvers")("cutoffTime,t",
                 boost::program_options::value<int>()->default_value(300),
                 "cutoff time (seconds)")(
      "logging_file",
      boost::program_options::value<std::string>()->default_value(""),
      "logging file");

  boost::program_options::variables_map vm;
  auto options = parse_command_line(argc, argv, options_description);
  boost::program_options::store(options, vm);
  boost::program_options::notify(vm);

  if (!vm["query"].as<std::string>().empty()) {
    if (vm["start"].as<int>() != -1 || vm["goal"].as<int>() != -1) {
      std::cerr
          << "query file and start/goal cannot be given at the same time !"
          << std::endl;
      return -1;
    }
  }
  if (vm["map"].as<std::string>().empty()) {
    std::cerr << "no directory for edge weights files was given" << std::endl;
    return -1;
  }

  auto parser = Parser(vm["map"].as<std::string>());
  auto adjecency_matrix = parser.default_graph();

  size_t source = vm["start"].as<int>();
  size_t target = vm["goal"].as<int>();
  std::string algorithm = vm["algorithm"].as<std::string>();
  double e1 = vm["e1"].as<double>();
  double e2 = vm["e2"].as<double>();
  ExperimentUtils::single_run(adjecency_matrix, source, target, algorithm,
                              {e1, e2});

  return 0;
}
