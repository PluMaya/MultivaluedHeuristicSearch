cmake_minimum_required(VERSION 3.28)
project(MultivaluedHeuristicSearch)

set(CMAKE_CXX_STANDARD 17)


set(BOOST_ROOT "C:/boost")
set(BOOST_INCLUDEDIR "${BOOST_ROOT}/include")
set(BOOST_LIBRARYDIR "${BOOST_ROOT}/lib")

# Find Boost
find_package(Boost REQUIRED COMPONENTS program_options log thread filesystem system)
include_directories(${Boost_INCLUDE_DIRS})


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")

include_directories("include")

# Collect all source files in src and tests directories
file(GLOB_RECURSE SRC_FILES "src/*.cpp" "src/**/*.cpp" "include/**/*.h" "include/*.h")
file(GLOB_RECURSE TEST_FILES "tests/*.cpp")

add_executable(MultivaluedHeuristicSearch ${SRC_FILES})

# Test executable
add_executable(RunAllTests ${TEST_FILES})

# Link Boost libraries
target_link_libraries(MultivaluedHeuristicSearch ${Boost_LIBRARIES})