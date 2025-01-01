//
// Created by crl on 13/12/2024.
//

#ifndef NAMOA_DR_H
#define NAMOA_DR_H
#include "abstract_solver.h"


class NAMOAdr: public AbstractSolver{

public:

    std::string get_solver_name() override { return "NAMOA*dr"; }

    explicit NAMOAdr(const AdjacencyMatrix &adj_matrix);

    void operator()(const size_t &source, const size_t &target,
                            const Heuristic &heuristic, SolutionSet &solutions,
                            unsigned int time_limit) override;
};




#endif //NAMOA_DR_H
