// //
// // Created by crl on 13/07/2024.
// //
//
//
// #include <iostream>
//
//
//
//
// void single_run_map(size_t graph_size, AdjacencyMatrix& graph, AdjacencyMatrix& inv_graph, size_t source, size_t target,
//                     std::ofstream& output, const std::string& algorithm, double eps, unsigned int time_limit)
// {
//     // Compute heuristic
//     std::cout << "Start Computing Heuristic" << std::endl;
//     ShortestPathHeuristic sp_heuristic(target, graph_size, inv_graph);
//     // sp_heuristic.set_all_to_zero();
//     std::cout << "Finish Computing Heuristic\n" << std::endl;
//
//     using std::placeholders::_1;
//     Heuristic heuristic = std::bind(&ShortestPathHeuristic::operator(), sp_heuristic, _1);
//
//     SolutionSet solutions;
//     int num_exp, num_gen;
//     auto runtime = std::clock();
//
//     std::unique_ptr<AbstractSolver> solver;
//     if (algorithm == "PPA")
//     {
//         Pair<double> eps_pair({eps, eps});
//         solver = std::make_unique<PPA>(graph, eps_pair, logger);
//     }
//     else if (algorithm == "BOA")
//     {
//         Pair<double> eps_pair({eps, eps});
//         solver = std::make_unique<BOAStar>(graph, eps_pair, logger);
//     }
//     else if (algorithm == "NAMOAdr")
//     {
//         EPS eps_vec(graph.get_num_of_objectives(), eps);
//         solver = std::make_unique<NAMOAdr>(graph, eps_vec, logger);
//         // ((ApexSearch*)solver.get())->set_merge_strategy(ms);
//     }
//     else if (algorithm == "Apex")
//     {
//         EPS eps_vec(graph.get_num_of_objectives(), eps);
//         solver = std::make_unique<ApexSearch>(graph, eps_vec, logger);
//         ((ApexSearch*)solver.get())->set_merge_strategy(ms);
//     }
//     else
//     {
//         std::cerr << "unknown solver name" << std::endl;
//         exit(-1);
//     }
//     auto start = std::clock();
//     (*solver)(source, target, heuristic, solutions, time_limit);
//     runtime = std::clock() - start;
//
//     std::cout << "Node expansion: " << solver->get_num_expansion() << std::endl;
//     std::cout << "Runtime: " << ((double)runtime) / CLOCKS_PER_SEC << std::endl;
//     num_exp = solver->get_num_expansion();
//     num_gen = solver->get_num_generation();
//     for (auto sol : solutions)
//     {
//         std::cout << *sol << std::endl;
//     }
//
//
//     output << algorithm << "-" << alg_variant << " (" << eps << ")" << "\t"
//         << source << "\t" << target << "\t"
//         << num_gen << "\t"
//         << num_exp << "\t"
//         << solutions.size() << "\t"
//         << (double)runtime / CLOCKS_PER_SEC
//         << std::endl;
//
//     std::cout << "-----End Single Example-----" << std::endl;
// }
//
// void single_run_map(size_t graph_size, std::vector<Edge>& edges, size_t source, size_t target, std::string output_file,
//                     std::string algorithm, MergeStrategy ms, LoggerPtr logger, double eps, int time_limit)
// {
//     AdjacencyMatrix graph(graph_size, edges);
//     AdjacencyMatrix inv_graph(graph_size, edges, true);
//     std::ofstream stats;
//     stats.open(output_path + output_file, std::fstream::app);
//
//     single_run_map(graph_size, graph, inv_graph, source, target, stats, algorithm, ms, logger, eps, time_limit);
// }
//
// void run_query(size_t graph_size, std::vector<Edge>& edges, std::string query_file, std::string output_file,
//                std::string algorithm, MergeStrategy ms, LoggerPtr logger, double eps, int time_limit)
// {
//     std::ofstream stats;
//     stats.open(output_path + output_file, std::fstream::app);
//
//
//     std::vector<std::pair<size_t, size_t>> queries;
//     if (load_queries(query_file, queries) == false)
//     {
//         std::cout << "Failed to load queries file" << std::endl;
//         return;
//     }
//
//     // Build graphs
//     AdjacencyMatrix graph(graph_size, edges);
//     AdjacencyMatrix inv_graph(graph_size, edges, true);
//
//     size_t query_count = 0;
//     for (auto iter = queries.begin(); iter != queries.end(); ++iter)
//     {
//         query_count++;
//         std::cout << "Started Query: " << query_count << "/" << queries.size() << std::endl;
//         size_t source = iter->first;
//         size_t target = iter->second;
//
//         single_run_map(graph_size, graph, inv_graph, source, target, stats, algorithm, ms, logger, eps, time_limit);
//     }
// }
