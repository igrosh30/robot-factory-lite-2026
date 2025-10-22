#include "heuristic.hpp"
#include <cmath>

namespace heuristic {
    int manhattan(const State& from, const State& to) {
        return std::abs(from.x() - to.x()) + std::abs(from.y() - to.y());
    }
    
    int euclidean(const State& from, const State& to) {
        return static_cast<int>(std::sqrt(
            std::pow(from.x() - to.x(), 2) + 
            std::pow(from.y() - to.y(), 2)
        ));
    }
    
    int zero(const State& from, const State& to) {
        return 0;  // Makes A* equivalent to uniform cost search
    }
}
