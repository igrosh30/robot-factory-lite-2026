#include "node.hpp"
#include <sstream>

State::State(int x, int y) : x_(x), y_(y) {}

std::string State::to_string() const {
    std::ostringstream oss;
    oss << "(" << x_ << "," << y_ << ")";
    return oss.str();
}

bool State::operator==(const State& other) const {
    return (x_ == other.x_) && (y_ == other.y_);
}

Node::Node(State state, Node* parent, char action, int path_cost, int depth)
    : state(state), parent(parent), action(action), 
      path_cost(path_cost), depth(depth) {}

bool Node::operator==(const Node& other) const {
    return state == other.state;
}

bool Node::operator<(const Node& other) const {
    return path_cost < other.path_cost;
}
