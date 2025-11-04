#ifndef HEURISTIC_HPP
#define HEURISTIC_HPP

#include "node.hpp"

namespace heuristic {
    int manhattan(const State& from, const State& to);
    int euclidean(const State& from, const State& to);
    int zero(const State& from, const State& to);  // For uniform cost search
}

#endif
