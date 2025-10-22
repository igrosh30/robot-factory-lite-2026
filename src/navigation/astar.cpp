#include "astar.hpp"
#include "heuristic.hpp"
#include <queue>
#include <iostream>
#include <algorithm>
#include <unordered_map>

std::vector<std::shared_ptr<Node>> AStar::expand(const Problem& problem, 
                                                const std::shared_ptr<Node>& node) {
    std::vector<std::shared_ptr<Node>> children;
    State state = node->state;
    
    for (char action : problem.actions(state)) {
        State next_state = problem.result(state, action);
        int cost = node->path_cost + problem.action_cost(state, action, next_state);
        auto child = std::make_shared<Node>(next_state, node.get(), action, cost, node->depth + 1);
        children.push_back(child);
    }
    return children;
}

int AStar::f(const std::shared_ptr<Node>& node, const Problem& problem) {
    int g = node->path_cost;
    return g + heuristic::manhattan(node->state, problem.get_goal());
}

std::shared_ptr<Node> AStar::search(const Problem& problem) {
    auto node = std::make_shared<Node>(problem.get_initial_state());
    
    // frontier <- priority queue ordered by f = g + h, with node as element
    auto compare = [&problem](const std::shared_ptr<Node>& a, 
                             const std::shared_ptr<Node>& b) {
        return f(a, problem) > f(b, problem);
    };
    
    std::priority_queue<
        std::shared_ptr<Node>,
        std::vector<std::shared_ptr<Node>>,
        decltype(compare)> frontier(compare);
    
    frontier.push(node);
    
    // reached <- lookup table
    std::unordered_map<State, std::shared_ptr<Node>> reached;
    reached[problem.get_initial_state()] = node;
    
    while (!frontier.empty()) {
        node = frontier.top();
        frontier.pop();
        
        if (problem.is_goal(node->state)) {
            return node;
        }
        
        for (auto child : expand(problem, node)) {
            State s = child->state;
            if (reached.find(s) == reached.end() || child->path_cost < reached[s]->path_cost) {
                reached[s] = child;
                frontier.push(child);
            }
        }
    }
    
    return nullptr;  // failure
}

std::vector<State> AStar::reconstruct_path(std::shared_ptr<Node> node) {
    std::vector<State> path;
    Node* curr = node.get();
    while (curr != nullptr) {
        path.push_back(curr->state);
        curr = curr->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}
