#ifndef ASTAR_HPP
#define ASTAR_HPP

#include "problem.hpp"
#include "node.hpp"
#include "heuristic.hpp"
#include <vector>
#include <memory>

class AStar {
public:
    static std::shared_ptr<Node> search(const Problem& problem);
    static std::vector<State> reconstruct_path(std::shared_ptr<Node> node);
    
private:
    static std::vector<std::shared_ptr<Node>> expand(const Problem& problem, 
                                                    const std::shared_ptr<Node>& node);
    static int f(const std::shared_ptr<Node>& node, const Problem& problem);
};
#endif
