#include "AStar.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <chrono>
#include <functional>
#include <random>
#include <stdexcept>
#include <vector>
#include <bitset>

using namespace std;

enum tile_type : uint8_t {
    current,
    solved,
    irrelevant
};

struct SlidePuzzleState
{
    uint8_t x0:4, y0:4;
    uint8_t x1:4, y1:4;
    uint8_t x2:4, y2:4;
    uint8_t padding = 0;

    struct Transition {
        uint8_t nx0:4, ny0:4;
    };


    SlidePuzzleState(uint8_t x0, uint8_t y0)
        : x0(x0), y0(y0), x1(15), y1(15), x2(15), y2(15) {}


    SlidePuzzleState(const SlidePuzzleState&) = default;
    SlidePuzzleState(const SlidePuzzleState *parent, Transition &transition)
    {
        *this = *parent;

        if (transition.nx0 == x1 && transition.ny0 == y1) {
            x1 = x0;
            y1 = y0;
        }
        else if (transition.nx0 == x2 && transition.ny0 == y2) {
            x2 = x0;
            y2 = y0;
        }
        x0 = transition.nx0;
        y0 = transition.ny0;
    }

    bool operator==(const SlidePuzzleState& other) const {
        return hash() == other.hash();
    }

    size_t hash() const {
        auto res = *reinterpret_cast<const uint32_t*>(this);

        if (x2 == 15) {
            return res & 0xffff;
        }
        return res;
    }

    struct Evaluator;

    void get_transitions(std::vector<Transition> &container, Evaluator* eval) const;


    struct Evaluator
    {
        int current_layer = 0;
        int clx = 0, cly = 0;
        bool vert = false;

        size_t N, M;
        size_t unsolved;

        vector<vector<tile_type>> tile_types;

        int dst_x1, dst_y1, dst_x2, dst_y2;

        Evaluator(size_t N) : N(N), M(N-1), unsolved(N*N) {
            for (int i = 0; i < N; i++)
                tile_types.emplace_back(N, irrelevant);
        }

        bool is_solved(vector<vector<int>>const& puzzle) {
            return unsolved < 4;
        }

        void increment_layer(SlidePuzzleState* state, vector<vector<int>>const& puzzle) {

            auto get_group_size = [this](size_t off) {
                return (N - off) == 3 ? 1 : 2;
            };
            dst_x1 = -1;
            dst_x2 = -1;

            for (int x = 0; x < N; x++) {
                for (int y = 0; y < N; y++)
                {
                    if (tile_types[x][y] == current)
                    {
                        tile_types[x][y] = solved;
                        unsolved--;
                    }
                }
            }

            if (unsolved == 4) {
                dst_x1 = M - 1, dst_y2 = M - 1;
                dst_x2 = M - 1, dst_y2 = M;
            }
            else {
                if (!vert) {
                    auto group_size = get_group_size(clx);

                    for (int i = 0; i < group_size; i++, clx++) {
                        if (clx == N) {
                            break;
                        }
                        tile_types[clx][current_layer] = current;

                        if (dst_x1 != -1) {
                            dst_x2 = clx;
                            dst_y2 = current_layer;
                        } else {
                            dst_x1 = clx;
                            dst_y1 = current_layer;
                        }
                    }
                    if (clx == N) {
                        clx = 0;
                        vert = true;
                        cly++;
                    }
                } else {
                    auto group_size = get_group_size(cly);

                    for (int i = 0; i < group_size; i++, cly++) {
                        if (cly == N) {
                            break;
                        }
                        tile_types[current_layer][cly] = current;

                        if (dst_x1 != -1) {
                            dst_x2 = current_layer;
                            dst_y2 = cly;
                        } else {
                            dst_x1 = current_layer;
                            dst_y1 = cly;
                        }
                    }

                    if (cly == N) {
                        vert = false;
                        current_layer++;
                        cly = current_layer;
                        clx = current_layer;
                    }
                }
            }


            auto xy1v = dst_x1 * N + dst_y1 + 1;
            auto xy2v = dst_x2 * N + dst_y2 + 1;

            for (int x = 0; x < N; x++) {
                for (int y = 0; y < N; y++) {
                    if (puzzle[x][y] == xy1v) {
                        state->x1 = x;
                        state->y1 = y;
                    }
                    else if (puzzle[x][y] == xy2v) {
                        state->x2 = x;
                        state->y2 = y;
                    }
                }
            }
            if (dst_x2 == -1) {
                state->x2 = 15;
                state->y2 = 15;
            }
        }

        uint32_t get_distance_to_target(const SlidePuzzleState* state) {
            uint32_t sum = 0;

            // Distance of first tile to target
            if (dst_x1 != -1) {
                sum += std::abs(dst_x1 - state->x1) + std::abs(dst_y1 - state->y1);
            }

            // Distance of second tile to target
            if (dst_x2 != -1) {
                sum += std::abs(dst_x2 - state->x2) + std::abs(dst_y2 - state->y2);
            }

            // If only 4 unsolved tiles are left, include the zero in the distance
            //if (unsolved == 4)
            {
                sum += N - state->x0 - 1 + N - state->y0 - 1;
            }

            return sum;
        }

        // Don't care about finding the shortest path.
        // The transition cost can be set to zero for a substantial performance gain.
        uint16_t get_transition_cost(const SlidePuzzleState* , const SlidePuzzleState* ) { return 0; }
    };
};


void SlidePuzzleState::get_transitions(std::vector<Transition> &container, Evaluator* eval) const  {
    auto parent = ((const AStar::AStar<SlidePuzzleState>::State*)(this))->parent;

    const auto& M = eval->M;


    auto try_add_chklock = [&](uint8_t nx0, uint8_t ny0) {
        if (eval->tile_types[nx0][ny0] != solved)
            container.push_back({nx0, ny0});
    };

    if (parent) {
        if (x0 > 0) {
            auto nx0 = x0 - 1;
            if (parent->x0 != nx0)
                try_add_chklock(nx0, y0);
        }
        if (x0 < M) {
            auto nx0 = x0 + 1;
            if (parent->x0 != nx0)
                try_add_chklock(nx0, y0);
        }

        if (y0 > 0) {
            auto ny0 = y0 - 1;
            if (parent->y0 != ny0)
                try_add_chklock(x0, ny0);
        }
        if (y0 < M) {
            auto ny0 = y0 + 1;
            if (parent->y0 != ny0)
                try_add_chklock(x0, ny0);
        }
    }
    else
    {
        if (x0 > 0) try_add_chklock(x0-1, y0);
        if (x0 < M) try_add_chklock(x0+1, y0);

        if (y0 > 0) try_add_chklock(x0, y0-1);
        if (y0 < M) try_add_chklock(x0, y0+1);
    }
}

template<typename Key, typename Value, typename Hash, typename Equal>
struct ArrayMap {
    std::array<Value, 1 << 24> values;
    std::bitset<1 << 24> initialized;

    Hash hash;

    Value& operator[](const Key& key) {
        auto idx = hash(key);
        auto& v = values[idx];
        if (!initialized[idx]) {
            initialized[idx] = true;
            v = Value();
        }
        return v;
    }

    void clear() {
        initialized.reset();
    }
};


template<typename T>
void append_result(vector<int>& result,
                                    vector<vector<int>>& puzzle,
                                    T& serach_result) {
    vector<decltype(serach_result.final_state)> states;

    for (auto& i : serach_result) {
        states.push_back(&i);
    }
    reverse(states.begin(), states.end());

    for (size_t i = 0; i < (states.size() - 1); i++) {
        swap(puzzle[states[i]->x0][states[i]->y0],
             puzzle[states[i+1]->x0][states[i+1]->y0]);

        result.push_back(puzzle[states[i]->x0][states[i]->y0]);
    }
};
vector<int> slide_puzzle(vector<vector<int>> Puzzle)
{
    using state_t = SlidePuzzleState;
    vector<int> result;
    static AStar::AStar<state_t, state_t::Evaluator, state_t::Transition, ArrayMap> astar;

    size_t N = Puzzle.size();

    typename state_t::Evaluator eval(N);

    uint8_t x0,y0;

    for (uint8_t x = 0; x < N; x++)
    {
        for (uint8_t y = 0; y < N; y++)
        {
            if (Puzzle[x][y] == 0)
            {
                x0 = x;
                y0 = y;
                goto found_zero;
            }
        }
    }

    found_zero:

    while (!eval.is_solved(Puzzle)) {
        state_t s(x0, y0);
        eval.increment_layer(&s, Puzzle);


        auto search_result = astar.search(&s, &eval);

        if (search_result.valid()) {
            append_result(result, Puzzle, search_result);
            if (eval.unsolved == 4) break;

            x0 = search_result.final_state->x0;
            y0 = search_result.final_state->y0;

        } else {
            for (auto& i : Puzzle) {
                for (auto& t : i) {
                    cout << int(t) << ',';
                }
                cout << endl;
            }
            cout << endl;
            throw std::logic_error("No solution found");
        }
    }

    return result;
}

using time_type = decltype(std::chrono::high_resolution_clock::now());

time_type get_time() {
    return std::chrono::high_resolution_clock::now();
}


void print_elapsed(time_type start, time_type end, string msg) {
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    cout << msg << "  Elapsed: " << duration.count() << "ms" << endl;
}




vector<vector<int>> generate_random_puzzle(int N, int ShuffleAmount=100)
{
    vector<vector<int>> res(N, vector<int>(N));

    for (int i = 0; i < N; i++)
        std::iota(res[i].begin(), res[i].end(), i*N + 1);

    res.back().back() = 0;

    int x0 = N-1, y0 = N-1;


    auto do_move = [&](int x, int y)
    {
        if (x < N && x >= 0 && y < N && y >= 0)
        {
            swap(res[x0][y0], res[x][y]);
            x0 = x;
            y0 = y;
        }
    };

    static random_device d;
    static mt19937 rng(d());
    static uniform_int_distribution<int> dist(0,3);

    dist(rng);

    int last_n = -1, n;

    for (int i = 0; i < ShuffleAmount; i++)
    {
        while ((n = dist(rng)) == last_n);
        switch(n)
        {
            case 0: do_move(x0-1,y0); last_n = 1;
                break;
            case 1: do_move(x0+1, y0); last_n = 0;
                break;
            case 2: do_move(x0, y0-1); last_n = 3;
                break;
            case 3: do_move(x0, y0+1); last_n = 2;
                break;
        }
    }

    return res;
}

template<typename T>
void PrintPuzzle(T const& puzzle) {
    for (int x = 0; x < puzzle.size(); x++) {
        for (int y = 0; y < puzzle[x].size(); y++) {
            cout << (int)puzzle[x][y] << ',';
        }
        cout << '\n';
    }
    cout << endl;
}


bool ValidateSolution(vector<vector<int>>& Puzzle, vector<int> const& Solution)
{
    int x0,y0,xi,yi;

    for (xi = 0; xi < Puzzle.size(); xi++)
    {
        for (yi = 0; yi < Puzzle[xi].size(); yi++)
        {
            if (Puzzle[xi][yi] == 0)
            {
                x0 = xi;
                y0 = yi;
                break;
            }
        }
    }

    auto make_move = [&](int x, int y, int value)
    {
        if (x < Puzzle.size() && x >= 0 && y < Puzzle.size() && y >= 0 && Puzzle[x][y] == value)
        {
            swap(Puzzle[x0][y0], Puzzle[x][y]);
            x0 = x;
            y0 = y;
            return true;
        }
        return false;
    };

    for (auto& i : Solution)
    {
        if (make_move(x0+1,y0, i)) continue;
        if (make_move(x0-1,y0, i)) continue;
        if (make_move(x0,y0+1, i)) continue;
        if (make_move(x0,y0-1, i)) continue;
    }


    int i = 1;
    for (int x = 0; x < Puzzle.size(); x++)
    {
        for (int y = 0; y < Puzzle.size(); y++)
        {
            if (Puzzle[x][y] != i++ && !(Puzzle[x][y] == 0 && x == (Puzzle.size()-1) && y == (Puzzle.size()-1))) return false;
        }
    }
    return true;
}


int main() {
    for (int i = 0; i < 100; i++) {
        auto puzzle = generate_random_puzzle(13, 10000);
        auto t1 = get_time();
        auto sol2 = slide_puzzle(puzzle);

        auto t2 = get_time();

        print_elapsed(t1, t2, "Array");

        bool valid = ValidateSolution(puzzle, sol2);
        cout << (valid ? "VALID" : "INVALID") << endl;
        if (!valid) {
            PrintPuzzle(puzzle);
            cout << endl;
        }
    }

    return 0;
}
