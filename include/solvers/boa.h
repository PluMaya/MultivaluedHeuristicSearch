//
// Created by crl on 14/07/2024.
//

#ifndef BOA_H
#define BOA_H
#include "abstract_solver.h"

class BOAStar : public AbstractSolver {

public:
  std::string get_solver_name() override { return "BOA*"; }

  explicit BOAStar(const AdjacencyMatrix &adj_matrix);

  void operator()(const size_t &source, const size_t &target,
                          const Heuristic &heuristic, SolutionSet &solutions,
                          unsigned int time_limit) override;
};

#endif // BOA_H
