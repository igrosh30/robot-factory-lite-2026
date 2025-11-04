#include <iostream>
#include "navigation/astar.hpp"
#include "navigation/problem.hpp"

// ANSI color codes
#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"

void test_paths() {
    std::cout << "=== Testing Path Finding ===" << std::endl;
    int num_fail = 0;
    std::vector<State> states = {
        State(0, 4), State(1, 4), State(2, 4), State(3, 4),
        State(2, 2), State(3, 2), State(2, 1), State(3, 1),
        State(5, 3), State(6, 3), State(5, 2), State(6, 2),
        State(5, 0), State(6, 0), State(7, 0), State(8, 0),
        State(0, 0)
    };

    for (State start : states) {
        for (State goal : states) {
            Problem problem(start, goal);
            if (start == goal) continue;
            auto result = AStar::search(problem);
            if (result) {
                std::cout << GREEN << "[Test PASSED] Found path from " << start.to_string() << " to " << goal.to_string() << std::endl;
                std::cout << YELLOW << "Path cost: " << result->path_cost << std::endl;
            } else {
                num_fail++;
                std::cout << RED << "[Test FAILED] No path found" << std::endl;
            }
        }
    }
    std::cout << RESET << "===== End Path Finding =====" << std::endl;
    std::cout << "Tests Failed: " << RED << num_fail << RESET << std::endl;
}

int main() {
    test_paths();
    return 0;
}
