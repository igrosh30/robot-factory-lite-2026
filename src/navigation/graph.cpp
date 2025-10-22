#include "graph.hpp"
#include <stdexcept>

const Graph& Graph::get_instance() {
    static Graph instance;
    return instance;
}

Graph::Graph() {
    all_states_ = {
        State(0, 0), State(0, 1), State(0, 2), State(0, 4),
        State(1, 4), 
        State(2, 1), State(2, 2), State(2, 4),
        State(3, 1), State(3, 2), State(3, 4),
        State(4, 0), State(4, 1), State(4, 2), State(4, 3), State(4, 4),
        State(5, 0), State(5, 2), State(5, 3),
        State(6, 0), State(6, 2), State(6, 3),
        State(7, 0),
        State(8, 0), State(8, 2), State(8, 3), State(8, 4)
    };
    
    neighbors_[State(0, 0)] = {
        {State(0, 1), 'u'},
        {State(4, 0), 'r'}
    };
    
    neighbors_[State(0, 1)] = {
        {State(0, 2), 'u'},
        {State(0, 0), 'd'},
        {State(2, 1), 'r'} 
    };
    
    neighbors_[State(0, 2)] = {
        {State(0, 4), 'u'},
        {State(0, 1), 'd'},
        {State(2, 2), 'r'}
    };
    
    neighbors_[State(0, 4)] = {
        {State(0, 2), 'd'},
        {State(1, 4), 'r'}
    };

    neighbors_[State(1, 4)] = {
        {State(0, 4), 'l'},
        {State(2, 4), 'r'}
    };

    neighbors_[State(2, 1)] = {
        {State(0, 1), 'l'}
    };

    neighbors_[State(2, 2)] = {
        {State(0, 2), 'l'}
    };

    neighbors_[State(2, 4)] = {
        {State(1, 4), 'l'},
        {State(3, 4), 'r'}
    };

    neighbors_[State(3, 1)] = {
        {State(4, 1), 'r'}
    };

    neighbors_[State(3, 2)] = {
        {State(4, 2), 'r'}
    };

    neighbors_[State(3, 4)] = {
        {State(2, 4), 'l'},
        {State(4, 4), 'r'}
    };

    neighbors_[State(4, 0)] = {
        {State(4, 1), 'u'},
        {State(0, 0), 'l'},
        {State(5, 0), 'r'}
    };

    neighbors_[State(4, 1)] = {
        {State(4, 2), 'u'},
        {State(4, 0), 'd'},
        {State(3, 1), 'l'}
    };

    neighbors_[State(4, 2)] = {
        {State(4, 3), 'u'},
        {State(4, 1), 'd'},
        {State(3, 2), 'l'},
        {State(5, 2), 'r'}
    };

    neighbors_[State(4, 3)] = {
        {State(4, 4), 'u'},
        {State(4, 2), 'd'},
        {State(5, 3), 'r'}
    };

    neighbors_[State(4, 4)] = {
        {State(4, 3), 'd'},
        {State(3, 4), 'l'},
        {State(8, 4), 'r'}
    };

    neighbors_[State(5, 0)] = {
        {State(4, 0), 'l'},
        {State(6, 0), 'r'}
    };

    neighbors_[State(5, 2)] = {
        {State(4, 2), 'l'},
    };

    neighbors_[State(5, 3)] = {
        {State(4, 3), 'l'},
    };

    neighbors_[State(6, 0)] = {
        {State(5, 0), 'l'},
        {State(7, 0), 'r'}
    };

    neighbors_[State(6, 2)] = {
        {State(8, 2), 'r'},
    };

    neighbors_[State(6, 3)] = {
        {State(8, 3), 'r'},
    };

    neighbors_[State(7, 0)] = {
        {State(6, 0), 'l'},
        {State(8, 0), 'r'}
    };

    neighbors_[State(8, 0)] = {
        {State(8, 2), 'u'},
        {State(7, 0), 'l'},
    };

    neighbors_[State(8, 2)] = {
        {State(8, 3), 'u'},
        {State(8, 0), 'd'},
        {State(6, 2), 'l'}
    };

    neighbors_[State(8, 3)] = {
        {State(8, 4), 'u'},
        {State(8, 2), 'd'},
        {State(6, 3), 'l'}
    };

    neighbors_[State(8, 4)] = {
        {State(8, 3), 'd'},
        {State(4, 4), 'l'},
    };

    // Make sure EVERY state in all_states_ has an entry in neighbors_
    // (even if it's an empty vector for dead ends)
    for (const State& state : all_states_) {
        if (neighbors_.find(state) == neighbors_.end()) {
            neighbors_[state] = {};  // No neighbors (dead end)
        }
    }
}

const std::vector<std::pair<State, char>>& Graph::get_neighbors(const State& state) const {
    auto it = neighbors_.find(state);
    if (it == neighbors_.end()) {
        throw std::out_of_range("State not in graph: " + state.to_string());
    }
    return it->second;
}

bool Graph::contains(const State& state) const {
    return neighbors_.find(state) != neighbors_.end();
}

const std::vector<State>& Graph::get_all_states() const {
    return all_states_;
}
