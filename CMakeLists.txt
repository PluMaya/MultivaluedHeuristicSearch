cmake_minimum_required(VERSION 3.28)
project(MultivaluedHeuristicSearch)

include_directories("include")

set(BOOST_ROOT "C:/boost")
set(BOOST_INCLUDEDIR "${BOOST_ROOT}/include")
set(BOOST_LIBRARYDIR "${BOOST_ROOT}/lib")

# Find Boost
find_package( Boost REQUIRED COMPONENTS program_options log thread filesystem system)
include_directories( ${Boost_INCLUDE_DIRS} )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")

set(CMAKE_CXX_STANDARD 14)

add_executable(MultivaluedHeuristicSearch src/main.cpp
        src/utils.cpp
        src/parser.cpp
        include/parser.h
        include/solvers/shortest_path_heuristic_computer.h
        include/data_structures/edge.h
        "include/data_structures/adjacency_matrix.h"
        src/data_structures/adjacency_matrix.cpp
        include/data_structures/node.h
        include/driver_utils/run.h
        src/driver_utils/run.cpp
        include/definitions.h
        include/data_structures/apex_path_pair.h
        src/data_structures/apex_path_pair.cpp
        include/solvers/abstract_solver.h
        include/solvers/boa.h
        src/solvers/boa.cpp
        test/solvers/test_boa.cpp
        include/solvers/apex.h
        include/data_structures/map_queue.h
        src/data_structures/map_queue.cpp
        src/solvers/apex.cpp
        test/test_parser.cpp
        src/solvers/shortest_path_heuristic_computer.cpp
        src/data_structures/node.cpp
        test/solvers/test_apex.cpp
)

target_link_libraries(MultivaluedHeuristicSearch ${Boost_LIBRARIES})
