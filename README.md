<h1> A* for C++11</h1>

<p>
A* is a pathfinding algorithm that finds the shortest path between two points in some state space.
</p>

<h3> Usage </h3>

The template must be provided with a state type, an evaluator type, a transition type and optionally a custom hashmap, with the 
default being std::unordered_map.

    template<typename UserState,
            typename Evaluator = typename UserState::Evaluator,
            typename Transition = typename UserState::Transition,
            template<typename Key, typename Value, typename StateHash, typename Equal, typename...>
            class HashMap=std::unordered_map
    >
    struct AStar

<h4> UserState </h4>
There are some restrictions on the state type, due to the substantial memory demands of the algorithm for large state spaces. <br>
<code>UserState</code> must be trivially destructible and trivially copyable.

Additionally, it must implement the following methods: <br>

    struct UserState {
        // Create a new state from a parent state and a transition
        UserState(const UserState* parent, Transition& transition);
        UserState(const UserState&) = default;
        bool operator==(const UserState& other) const;
        size_t get_hash() const;
        void get_transitions(std::vector<Transition>& transitions, Evaluator* evaluator) const;
    };
    
Inside of AStar, a new state type is derived from UserState which has some A* specific members
like G and F values, as well as a pointer to the parent state, which will later be used to retrieve the solution. <br>
Its memory overhead is 24 bytes.

<h4> Evaluator </h4>
The evaluator is used to evaluate the state. It must implement the following two methods: <br>

    struct Evaluator {
        // The distance between `state` and the target state (H)
        // If the state is the target state, then the distance is 0
        uint32_t get_distance_to_target(const UserState* state); 
    
        // The cost of transitioning from `s0` to `s1` (G increment)
        uint16_t get_transition_cost(const UserState* s0, const UserState* s1);
    };


<h4> Transition </h4>
There are no restrictions on the Transition type, it is not used in any way except to be fed
back to the UserState constructor for the generation of the next state.


<h4> HashMap </h4>
The hashmap is used to find transpositions, states that are equivalent to each other, reached by different paths. 
It must implement the following two methods: <br>

    Value& operator[](const Key& key);
    void clear();

Both key and value will always be of type <code> AStar::AStar<...>::State*; </code>

Using std::unordered_map seems to be a significant bottleneck, using a custom hashmap may be preferable.


<h4> Searching </h4>
    
    // Searches through the state space until it finds a node with H == 0 or all nodes are exhausted
    Solution search(const UserState *start_, Evaluator *evaluator_)

<h4> Solution </h4>
The <code>Solution</code> struct has a single member <code> State* final_state </code> <br>
It provides a forward iterator which can be used to iterate through the parent states, down to the starting state. <br>
If no valid solution was found, then <code> final_state == nullptr</code> and <code>begin() == end()</code>.

<h4> Example </h4>
slide_puzzle.cpp contains an example of how to efficiently solve the slide puzzle problem using this algorithm.

