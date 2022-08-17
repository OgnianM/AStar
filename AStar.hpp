/*
    MIT License

    Copyright (c) 2022 Ognyan Mirev

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
 */

#ifndef ASTAR_HPP
#define ASTAR_HPP
#include <vector>
#include <queue>
#include <unordered_map>
#include <cstddef>
#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <chrono>

namespace AStar {
    template<typename T, size_t ElementsPerBlock = 1 << 16>
    struct MemoryManager {

        MemoryManager() : CurrentBlock(0), IndexInBlock(0) {
            allocate_block();
        }

        ~MemoryManager() {
            for (auto &i: MemoryBlocks)
                free(i);
        }

        inline T *reserve(size_t count) {
            if ((ElementsPerBlock - IndexInBlock) < count) {
                allocate_block();
                CurrentBlock++;
                IndexInBlock = 0;
            }

            auto ptr = MemoryBlocks[CurrentBlock] + IndexInBlock;
            IndexInBlock += count;
            return ptr;
        }

        inline void unreserve(size_t count) {
            IndexInBlock -= count;
        }

        inline void clear() {
            CurrentBlock = 0;
            IndexInBlock = 0;
        }

    private:

        size_t CurrentBlock, IndexInBlock;
        std::vector<T *> MemoryBlocks;

        void allocate_block() {
            auto ptr = malloc(sizeof(T) * ElementsPerBlock);
            if (!ptr) throw std::bad_alloc();
            MemoryBlocks.push_back(static_cast<T *>(ptr));
        }
    };

    template<typename UserState,
            typename Evaluator = typename UserState::Evaluator,
            typename Transition = typename UserState::Transition,
            template<typename Key, typename Value, typename StateHash, typename Equal, typename...>
            class HashMap=std::unordered_map
    >
    struct AStar {

        struct State : public UserState {
            // 16 bytes
            State *parent, *children;

            // 4 bytes
            uint32_t F;

            // 2 bytes
            uint16_t G;

            // 2 bytes
            uint16_t n_children:15;
            bool expanded:1;


            // 16 + 2 + 2 + 4 = 24 bytes per node


            struct StatePrioCompare {
                bool operator()(const State *a, const State *b) { return a->F > b->F; }
            };

            typedef typename std::priority_queue<State *,
                std::vector<State *>, StatePrioCompare> PriorityQueue;

            State() = delete;
            ~State() = delete;
            State(const State &) = default;

            State(const UserState *start_node, Evaluator *evaluator) :
                    UserState(*start_node),
                    G(0),
                    F(evaluator->get_distance_to_target(this)),
                    parent(nullptr),
                    n_children(0),
                    expanded(false)
                    {}

            State(State *parent, Transition &transition, Evaluator* evaluator) :
                    UserState(parent, transition),
                    parent(parent),
                    n_children(0),
                    expanded(false) {
                G = parent->G + evaluator->get_transition_cost(parent, this);
                F = evaluator->get_distance_to_target(this) + G;
            }

            inline State *begin() {
                return children;
            }

            inline State *end() {
                return children + n_children;
            }

            inline uint32_t GetH() const {
                return F - G;
            }

            inline void update_G(uint32_t new_G, AStar* inst) {
                auto& nodes_to_update = inst->NodesToUpdate;
                auto& NodeQueue = inst->NodeQueue;

                for (auto &i: *this) {
                    i.parent = this;
                    nodes_to_update.push_back(&i);
                }

                auto G_offset = G - new_G;
                F -= G_offset;
                G = new_G;

                if (!expanded)
                    NodeQueue.push(this);
                else while (!nodes_to_update.empty()) {
                    auto node = nodes_to_update.back();
                    nodes_to_update.pop_back();

                    node->G -= G_offset;
                    node->F -= G_offset;

                    if (!node->expanded)
                        NodeQueue.push(node);
                    else for (auto &i: *node)
                        nodes_to_update.push_back(&i);
                }
            }

            /*
             * If the current path to the child is has a lower G than the previous path,
             * the child node is "adopted"
             */
            inline void adopt(State *old_base, State *new_base, AStar* inst) {
                auto NewG = new_base->G;
                *new_base = *old_base;
                old_base->n_children = 0;
                old_base->expanded = true;

                new_base->parent = this;
                new_base->update_G(NewG, inst);
            }

            template<bool AtRoot = false>
            void expand(AStar *inst) {
                if (expanded) return;
                expanded = true;

                inst->Transitions.clear();

                this->get_transitions(inst->Transitions, inst->evaluator);
                if (inst->Transitions.empty()) return;

                n_children = inst->Transitions.size();
                children = inst->Memory.reserve(n_children);


                auto current_child = children;

                for (auto &i: inst->Transitions) {
                    auto ptr = new(current_child) State(this, i, inst->evaluator);
                    auto &table_entry = inst->TranspositionTable[ptr];


                    if (!AtRoot && table_entry != nullptr) {
                        if (table_entry->G > ptr->G) {
                            adopt(table_entry, current_child, inst);
                            table_entry = current_child;
                        } else {
                            n_children--;
                            current_child--;
                            inst->Memory.unreserve(1);
                        }
                    } else {
                        table_entry = ptr;
                        inst->NodeQueue.push(ptr);
                    }
                    current_child++;
                }
            }
        };




        AStar() {
            static_assert(std::is_trivially_destructible<UserState>::value,
                    "UserState must be trivially destructible");
            static_assert(std::is_trivially_copyable<UserState>::value,
                    "UserState must be trivially copyable");
            NodesToUpdate.reserve(2048);
        }

        struct Solution {
            State *final_state;

            Solution(State *final_state) : final_state(final_state) {}
            Solution() : final_state(nullptr) {}

            struct SolutionIterator {
                State *current_state;

                void operator++() {
                    current_state = current_state->parent;
                }

                State &operator*() {
                    return *current_state;
                }

                bool operator==(const SolutionIterator &other) {
                    return current_state == other.current_state;
                }

                bool operator!=(const SolutionIterator &other) {
                    return current_state != other.current_state;
                }
            };

            SolutionIterator begin() const {
                return {final_state};
            }

            SolutionIterator end() const {
                return {nullptr};
            }

            bool valid() const {
                return final_state != nullptr;
            }
        };


        // Searches through the state space until it finds a node with H == 0 or all nodes are exhausted
        Solution search(const UserState *start_, Evaluator *evaluator_) {
            try {
                TranspositionTable.clear();
                NodeQueue.clear();
                Memory.clear();

                this->evaluator = evaluator_;
                this->start = new (Memory.reserve(1)) State(start_, evaluator_);

                this->start->template expand<true>(this);

                if (start->F == 0) {
                    return {start};
                };

                while (!NodeQueue.empty())
                {
                    auto next = NodeQueue.top();
                    NodeQueue.pop();

                    // H == 0 means we found a solution
                    if (next->F == next->G)
                        return next;

                    next->expand(this);
                }
            }
            catch (const std::exception& e) {
                std::cout << e.what() << std::endl;
            }
            return {};
        }


    protected:
        struct StateHash {
            size_t operator()(const State *state) const {
                return state->hash();
            }
        };

        struct EqualityCheck {
            bool operator()(const State *a, const State *b) const {
                return *a == *b;
            }
        };

        struct PriorityQueue : public State::PriorityQueue {
            void clear() {
                this->c.clear();
            }
        };

        //region Containers
        HashMap<State*, State *, StateHash, EqualityCheck> TranspositionTable;
        PriorityQueue NodeQueue;
        MemoryManager<State> Memory;
        std::vector<State*> NodesToUpdate;
        std::vector<Transition> Transitions;
        //endregion

        //region Parameters
        State *start;
        Evaluator *evaluator;
        //endregion
    };
}


#endif //ASTAR_HPP
