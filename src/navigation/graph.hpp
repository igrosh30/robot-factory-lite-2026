#ifndef GRAPH_HPP
#define GRAPH_HPP

#include "node.hpp"
#include <vector>
#include <unordered_map>

class Graph {
public:
    static const Graph& get_instance();
    
    const std::vector<std::pair<State, char>>& get_neighbors(const State& state) const;
    bool contains(const State& state) const;
    
    const std::vector<State>& get_all_states() const;

private:
    Graph();  // Private constructor - builds the fixed graph
    
    std::unordered_map<State, std::vector<std::pair<State, char>>> neighbors_;
    std::vector<State> all_states_;
};

#endif
