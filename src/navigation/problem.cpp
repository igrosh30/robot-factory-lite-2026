#include "problem.hpp"
#include <stdexcept>

Problem::Problem(State initial, State goal) 
    : initial_(initial), goal_(goal), graph_(Graph::get_instance()) {
    
    // Validate that initial and goal states exist in our graph
    if (!graph_.contains(initial)) {
        throw std::invalid_argument("Initial state not in graph: " + initial.to_string());
    }
    if (!graph_.contains(goal)) {
        throw std::invalid_argument("Goal state not in graph: " + goal.to_string());
    }
}

std::vector<char> Problem::actions(const State& state) const {
    if (!graph_.contains(state)) {
        throw std::invalid_argument("State not in graph: " + state.to_string());
    }
    
    const auto& neighbors = graph_.get_neighbors(state);
    std::vector<char> valid_actions;
    
    for (const auto& [neighbor_state, action_char] : neighbors) {
        // Convert char to string
        valid_actions.push_back(action_char);
    }
    
    return valid_actions;
}

State Problem::result(const State& state, const char& action) const {
    if (!graph_.contains(state)) {
        throw std::invalid_argument("State not in graph: " + state.to_string());
    }
    
    const auto& neighbors = graph_.get_neighbors(state);
    for (const auto& [neighbor_state, neighbor_action] : neighbors) {
        if (neighbor_action == action) {
            return neighbor_state;
        }
    }
    
    throw std::invalid_argument("Invalid action " + std::string(1, action) + " from state: " + state.to_string());
}

int Problem::action_cost(const State& state, const char& action, const State& result_state) const {
    // Uniform cost - always 1 for moving to adjacent state
    return 1;
}

bool Problem::is_goal(const State& state) const {
    return state == goal_;
}

State Problem::get_initial_state() const {
    return initial_;
}

State Problem::get_goal() const {
    return goal_;
}

