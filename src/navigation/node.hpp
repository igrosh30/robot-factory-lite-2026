#ifndef NODE_HPP
#define NODE_HPP
#include <string>

class State {
    private:
        int x_, y_;

    public:
        State(int x = 0, int y = 0);

        int x() const { return x_; }
        int y() const { return y_; }

        std::string to_string() const;
        bool operator==(const State& other) const;
};

class Node {
    public:
        State state;
        Node* parent;
        char action;
        int path_cost;
        int depth;

        Node(State state, Node* parent = NULL, char action = 0, int path_cost = 0, int depth = 0);

        bool operator==(const Node& other) const;
        bool operator<(const Node& other) const;
};

namespace std {
    template<>
    struct hash<State> {
        size_t operator()(const State& s) const {
            return hash<int>()(s.x()) ^ (hash<int>()(s.y()) << 1);
        }
    };
}
#endif
