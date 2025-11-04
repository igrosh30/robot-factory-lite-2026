#include <vector>
#include "node.hpp"
#include "graph.hpp"
#ifndef PROBLEM_HPP
#define PROBLEM_HPP

class Problem {
    private:
        State initial_;
        State goal_;
        const Graph& graph_;

    public:
        Problem(State initial, State goal);

        std::vector<char> actions(const State& state) const;
        State result(const State& state, const char& action) const;
        int action_cost(const State& state, const char& action, const State& result_state) const;
        bool is_goal(const State& state) const;
        
        State get_initial_state() const;
        State get_goal() const;
};

#endif
