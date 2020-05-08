#include <iostream>
#include <utility>
#include <vector>
#include <set>
#include <unordered_map>
#include <cstdint>
#include <limits>
#include <string>
#include <list>
#include <queue>
#include <deque>
#include <unordered_set>
#include "problem.h"

#ifdef TIME_EXECUTION
#include <chrono>
#endif

using std::cout;
using std::cerr;
using std::endl;
using std::vector;
using std::set;
using std::unordered_map;
using std::string;
using std::pair;
using std::list;
using std::queue;
using std::unordered_set;
using std::deque;

class State;

typedef int32_t FACT_COST_TYPE;

class LabeledGraph {
public:
    size_t vertices;
    vector<deque<pair<size_t, int>>> adjacency_out; 
    vector<deque<pair<size_t, int>>> adjacency_in;

    explicit LabeledGraph(size_t vertices) : vertices(vertices), adjacency_out(vertices), adjacency_in(vertices) {}

    LabeledGraph(LabeledGraph &&other) = default;

    void add_edge(size_t from, size_t to, int operator_label) {
        adjacency_out[from].emplace_back(to, operator_label);
        adjacency_in[to].emplace_back(from, operator_label);
    }
};

class StripsOperator {
public:
    string name;
    int cost;
    int lm_cost;
    vector<int> preconditions;
    vector<int> add_effects;
    vector<int> del_effects;

    explicit StripsOperator(strips_operator_t &op) :
            name(op.name),
            cost{op.cost},
            lm_cost{op.cost},
            preconditions(op.pre, op.pre + op.pre_size),
            add_effects(op.add_eff, op.add_eff + op.add_eff_size),
            del_effects(op.del_eff, op.del_eff + op.del_eff_size) {}

    StripsOperator(string name, int cost) : name{std::move(name)}, cost{cost}, lm_cost{cost} {}

    void reset_lm_cost() {
        lm_cost = cost;
    }
};

class Strips {
private:
    bool extended{false};
public:
    int num_facts;
    vector<string> fact_names;
    vector<int> initial_state;
    vector<int> goal_state;
    vector<int> lm_init;
    vector<int> lm_goal;
    vector<StripsOperator> operators;

    explicit Strips(strips_t &strips) :
            num_facts{strips.num_facts},
            fact_names(strips.fact_names, strips.fact_names + num_facts),
            initial_state(strips.init, strips.init + strips.init_size),
            goal_state(strips.goal, strips.goal + strips.goal_size),
            operators(strips.operators, strips.operators + strips.num_operators)
    {}

    /**
     * Extends the Strips problem to be used with LM-Cut
     * Adds a new goal_fact, with number original num_facts and a new init fact with number original num_facts + 1
     * num_facts is then incremented by 2
     */
    void extend_for_lm() {
        if (not extended) {
            extended = true;
            lm_goal.emplace_back(num_facts++);
            fact_names.emplace_back("LM-Goal-fact");
            lm_init.emplace_back(num_facts++);
            fact_names.emplace_back("LM-Init-fact");

            operators.emplace_back("new_goal_op", 0);
            StripsOperator &lm_goal_op = operators.back();
            lm_goal_op.add_effects.emplace_back(lm_goal.front());
            lm_goal_op.preconditions.insert(lm_goal_op.preconditions.end(), goal_state.begin(), goal_state.end());

            operators.emplace_back("new_init_op", 0);
            StripsOperator &lm_init_op = operators.back();
            lm_init_op.preconditions.emplace_back(lm_init.front());
        }
    }
};

uint64_t lmcut(const State &state,
               Strips &problem,
               const vector<vector<bool>> &op_preconditions,
               const vector<bool> &original_init_facts,
               const vector<bool> &lm_init_facts);

vector<vector<bool>> create_op_preconditions(const Strips &problem);

vector<bool> find_zero_cost_reachable_from_end(LabeledGraph &graph, int end_fact, const Strips &problem);

unordered_set<int>
find_landmarks_given_N_star(LabeledGraph &graph, int start_fact, Strips &problem, vector<bool> &N_star);

set<int>
find_landmarks_given_N_star_set(LabeledGraph &graph, int start_fact, Strips &problem, vector<bool> &N_star);

vector<int>
find_landmarks_given_N_star_vect(LabeledGraph &graph, int start_fact, Strips &problem, vector<bool> &N_star);

deque<int>
find_landmarks_given_N_star_deque(LabeledGraph &graph, int start_fact, Strips &problem, vector<bool> &N_star);

vector<bool> init_facts(size_t num_facts, const vector<int> &facts_lst) {
    vector<bool> facts(num_facts, false);
    for (auto fact : facts_lst) {
        facts[fact] = true;
    }
    return facts;
}

vector<bool> init_facts(const Strips &problem) {
    return init_facts(problem.num_facts, problem.initial_state);
}

bool is_goal_facts_subset(const vector<bool> &facts, const vector<int> &goal_state) {
    for (auto goal_fact : goal_state) {
        if (not facts[goal_fact]) {
            return false;
        }
    }
    return true;
}

bool is_goal_facts_subset(const vector<bool> &facts, const Strips &problem) {
    return is_goal_facts_subset(facts, problem.goal_state);
}

class State {
private:
    State(vector<bool> succ_facts, vector<int> succ_path, uint64_t new_cost) :
            facts(std::move(succ_facts)),
            path(std::move(succ_path)),
            cost{new_cost},
            facts_hash{ std::hash<vector<bool>>{}(facts) } {}

public:
    vector<bool> facts;
    vector<int> path;
    uint64_t cost;
    size_t facts_hash;

    explicit State(const Strips &problem) :
            facts( init_facts(problem) ),
            cost{0},
            facts_hash{ std::hash<vector<bool>>{}(facts) } {};

    bool is_operator_applicable(const strips_operator_t &op) const {
        bool applicable = true;
        for (size_t pre_idx = 0; pre_idx < op.pre_size; pre_idx++) {
            if (not facts[op.pre[pre_idx]]) {
                applicable = false;
                break;
            }
        }
        return applicable;
    }

    bool is_operator_applicable(const StripsOperator &op) const {
        bool applicable = true;
        for (auto precondition_fact : op.preconditions) {
            if (not facts[precondition_fact]) {
                applicable = false;
                break;
            }
        }
        return applicable;
    }

    bool is_goal_state(const Strips &problem) const {
        return is_goal_facts_subset(facts, problem);
    }

    State apply_operator(const StripsOperator &op, const int operator_idx) const {

#ifdef MY_DEBUG
        if(! is_operator_applicable(op)) {
            cerr << "Can't apply operator!!!" << endl;
            exit(-1);
        }
#endif

        vector<bool> new_facts(facts);
        vector<int> new_path(path);

        uint64_t new_cost = cost + op.cost;
        new_path.emplace_back(operator_idx);

        for (auto del_fact : op.del_effects) {
            new_facts[del_fact] = false;
        }

        for (auto add_fact : op.add_effects) {
            new_facts[add_fact] = true;
        }

        State succ(std::move(new_facts), std::move(new_path), new_cost);
        return succ;
    }
};

/**
 * A* using pairs of <cost, hash> and a supplementary map of <hash, State object>
 * @param problem
 * @return
 */
pair<State, uint64_t> get_plan_lm(Strips &problem) {

    problem.extend_for_lm();

    const vector<vector<bool>> op_preconditions = create_op_preconditions(problem);
    const vector<bool> original_init_facts = init_facts(problem.num_facts, problem.initial_state);
    const vector<bool> lm_init_facts = init_facts(problem.num_facts, problem.lm_init);

    unordered_map<size_t, uint64_t> distance;
    unordered_map<size_t, State> states;
    set<std::pair<uint64_t, size_t>> queue;

    State start_state(problem);
    uint64_t initial_lmcut = lmcut(start_state, problem, op_preconditions, original_init_facts, lm_init_facts);
    distance[start_state.facts_hash] = 0;
    queue.insert(std::make_pair(0, start_state.facts_hash));
    states.insert(std::make_pair(start_state.facts_hash, std::move(start_state)));

    const size_t op_count = problem.operators.size();

    while (not queue.empty()) {
        State cur_state = std::move(states.find(queue.begin()->second)->second);
        states.erase(cur_state.facts_hash);
        queue.erase(queue.begin());

        if (cur_state.is_goal_state(problem)) {
            return std::make_pair(std::move(cur_state), initial_lmcut);
        }


        for (size_t op_idx = 0; op_idx < op_count; op_idx++) {
            StripsOperator &current_operator = problem.operators[op_idx];

            if (cur_state.is_operator_applicable(current_operator)) {
                State next_state = cur_state.apply_operator(current_operator, op_idx);
                uint64_t heuristic_val = lmcut(next_state, problem, op_preconditions, original_init_facts, lm_init_facts);
                if (heuristic_val < std::numeric_limits<FACT_COST_TYPE>::max()) {
                    uint64_t predicted_cost = next_state.cost + heuristic_val;
                    if (predicted_cost < heuristic_val) {
                        predicted_cost = std::numeric_limits<uint64_t>::max();
                    }
                    auto appearance_it = distance.find(next_state.facts_hash);

                    uint64_t cur_cost = appearance_it == distance.end() ? std::numeric_limits<uint64_t>::max() : appearance_it->second;
                    if (cur_cost == std::numeric_limits<uint64_t>::max() || cur_cost > predicted_cost) {
                        //since f(s) = g(s) + h(s), and since h(s) for a state never changes
                        //and default distance value is infinity, then we don't need to check for closed set (it is implied)
                        //and f(s) < previously saved f(s) is equivalent to g(s) + const < distance(s) + const
                        //then reopening will work as well
                        if (cur_cost != std::numeric_limits<uint64_t>::max()) {
                            auto it = queue.find(std::make_pair(cur_cost, next_state.facts_hash));
                            if (it != queue.end()) {
                                queue.erase(it); // element in priority queue, value will be updated
                            } // else : element in implied closed set, will be reopened
                        }

                        distance[next_state.facts_hash] = predicted_cost;
                        queue.emplace(predicted_cost, next_state.facts_hash);

                        auto it = states.find(next_state.facts_hash);
                        if ( it != states.end() ) {
                            it->second = std::move(next_state);
                        } else {
                            states.emplace(next_state.facts_hash, std::move(next_state));
                        }
                    }
                }
            }
        }
    }

    State not_found = State(problem);
    return std::make_pair(std::move(not_found), initial_lmcut);
}

vector<vector<bool>> create_op_preconditions(const Strips &problem) {
    const size_t operator_count = problem.operators.size();
    vector<vector<bool>> op_preconditions(operator_count, vector<bool>(problem.num_facts, false));
    for (size_t op_idx = 0; op_idx < operator_count; op_idx++) {
        const StripsOperator &current_operator = problem.operators[op_idx];
        for (int pre_fact : current_operator.preconditions) {
            op_preconditions[op_idx][pre_fact] = true;
        }
    }
    return op_preconditions;
}


pair<FACT_COST_TYPE , vector<FACT_COST_TYPE>> hmax_for_lm(const vector<bool> &state_facts, const Strips &problem, const vector<int>& goal, const vector<vector<bool>> &op_preconditions) {

    vector<FACT_COST_TYPE> delta(problem.num_facts, std::numeric_limits<FACT_COST_TYPE>::max());
    for (int fact_idx = 0; fact_idx < problem.num_facts; ++fact_idx) {
        if (state_facts[fact_idx]) {
            delta[fact_idx] = 0;
        }
    }

    const size_t operator_count = problem.operators.size();

    vector<int> U(operator_count, -1);

    for (size_t op_idx = 0; op_idx < operator_count; ++op_idx) {
        const StripsOperator &current_operator = problem.operators[op_idx];

        int pre_size = current_operator.preconditions.size();
        U[op_idx] = pre_size;

        if (pre_size == 0) {
            for (size_t current_add_fact : current_operator.add_effects) {
                delta[current_add_fact] = std::min(delta[current_add_fact], current_operator.lm_cost);
            }
        }
    }

    vector<bool> C(problem.num_facts, false);

    set<std::pair<FACT_COST_TYPE, int>> queue;
    for (int fact_idx = 0; fact_idx < problem.num_facts; fact_idx++) {
        queue.emplace(delta[fact_idx], fact_idx);
    }

    while( not is_goal_facts_subset(C, goal)) {
        int k = queue.begin()->second;
        queue.erase(queue.begin());
        C[k] = true;

        for (size_t op_idx = 0; op_idx < operator_count; ++op_idx) {
            if (op_preconditions[op_idx][k]) {
                const StripsOperator &current_operator = problem.operators[op_idx];
                U[op_idx]--;

                if (U[op_idx] == 0) {
                    for (size_t cur_add_fact : current_operator.add_effects) {
                        FACT_COST_TYPE potentialCost = current_operator.lm_cost + delta[k];
                        if (potentialCost < 0) {
                            potentialCost = std::numeric_limits<FACT_COST_TYPE>::max();
                        }

                        if (delta[cur_add_fact] > potentialCost) {
                            if (not C[cur_add_fact]) {
                                queue.erase(queue.find(std::make_pair(delta[cur_add_fact], cur_add_fact)));
                                queue.emplace(potentialCost, cur_add_fact);
                            }
                            delta[cur_add_fact] = potentialCost;
                        }
                    }
                }
            }
        }
    }

    FACT_COST_TYPE heur{std::numeric_limits<FACT_COST_TYPE>::min()};
    for (int goal_fact : goal){
        heur = std::max(heur, delta[goal_fact]);
    }
    return std::make_pair(heur, std::move(delta));
}

vector<int> find_landmarks_backwards_bfs(LabeledGraph &graph, int end_fact, Strips &problem);

uint64_t lmcut(const State &state,
               Strips &problem,
               const vector<vector<bool>> &op_preconditions,
               const vector<bool> &original_init_facts,
               const vector<bool> &lm_init_facts) {

    static unordered_map<size_t, uint64_t> cache;
    static FACT_COST_TYPE orig_init_hmax{std::numeric_limits<FACT_COST_TYPE>::max()};
    static bool orig_init_hmax_computed{false};

    if(cache.find(state.facts_hash) != cache.end()) {
        return cache[state.facts_hash];
    }

    for (StripsOperator &anOperator : problem.operators) {
        anOperator.reset_lm_cost();
    }

    if (not orig_init_hmax_computed) {
        orig_init_hmax_computed = true;
        pair<int, vector<int>> orig_hmax = hmax_for_lm(original_init_facts, problem, problem.goal_state, op_preconditions);
        orig_init_hmax = orig_hmax.first;
    }

    if (orig_init_hmax == std::numeric_limits<FACT_COST_TYPE>::max()) {
        cache[state.facts_hash] = std::numeric_limits<FACT_COST_TYPE>::max();
        return std::numeric_limits<FACT_COST_TYPE>::max();
    }

    StripsOperator &init_op = problem.operators.back();
    init_op.add_effects.clear();
    for (int fact_idx = 0; fact_idx < problem.num_facts; fact_idx++) {
        if (state.facts[fact_idx]) {
            init_op.add_effects.emplace_back(fact_idx);
        }
    }

    uint64_t h_lmcut{0};
    const int op_count = problem.operators.size();
    int hmax_val{};
    vector<int> delta;
    std::tie(hmax_val, delta) = hmax_for_lm(lm_init_facts, problem, problem.lm_goal, op_preconditions);
    while (hmax_val > 0 and hmax_val < std::numeric_limits<int>::max()) {
        LabeledGraph justification(problem.num_facts);

        for (int op_idx = 0; op_idx < op_count; op_idx++) {
            StripsOperator &op = problem.operators[op_idx];
            FACT_COST_TYPE max_cost = std::numeric_limits<FACT_COST_TYPE>::min();
            int op_support{-1};
            for (int pre : op.preconditions) {
                if (delta[pre] == std::numeric_limits<FACT_COST_TYPE>::max()) {
                    op_support = -1;
                    break;
                }
                if (delta[pre] >= max_cost) {
                    max_cost = delta[pre];
                    op_support = pre;
                }
            }
            if (op_support >= 0) {
                for (int add_fact : op.add_effects) {
                    justification.add_edge(op_support, add_fact, op_idx);
                }
            }
        }

        vector<bool> N_star = find_zero_cost_reachable_from_end(justification, problem.num_facts-2, problem);
        auto landmarkss = find_landmarks_given_N_star(justification, problem.num_facts - 1, problem, N_star);
//        auto landmarks = find_landmarks_backwards_bfs(justification, problem.num_facts - 2, problem);

        vector<int> landmarks;
        landmarks.reserve(landmarkss.size());

        int min_landmark_cost{std::numeric_limits<int>::max()};
        for (int landmark : landmarkss) {
            StripsOperator &landmarkOp = problem.operators[landmark];
            if (landmarkOp.lm_cost < min_landmark_cost) {
                min_landmark_cost = landmarkOp.lm_cost;
            }
            landmarks.emplace_back(landmark);
        }
        
        h_lmcut += min_landmark_cost;
        for (int landmark: landmarks) {
            problem.operators[landmark].lm_cost -= min_landmark_cost;
        }

        std::tie(hmax_val, delta) = hmax_for_lm(lm_init_facts, problem, problem.lm_goal, op_preconditions);
    }

    if (hmax_val >= std::numeric_limits<int>::max() or hmax_val < 0) {
        h_lmcut = std::numeric_limits<int>::max();
    }
    cache[state.facts_hash] = h_lmcut;
    return h_lmcut;

}

unordered_set<int>
find_landmarks_given_N_star(LabeledGraph &graph, int start_fact, Strips &problem, vector<bool> &N_star) {
    vector<bool> visited(problem.num_facts, false);
    unordered_set<int> landmarks;
    queue<int> queue;
    visited[start_fact] = true;
    queue.push(start_fact);

    while (not queue.empty()) {
        int cur_fact = queue.front();
        queue.pop();

        for (auto& neigh_fact_op_pair : graph.adjacency_out[cur_fact]) {
            int neigh_fact = neigh_fact_op_pair.first;
            int op_idx = neigh_fact_op_pair.second;
            if (N_star[neigh_fact]) {
                landmarks.insert(op_idx);
            } else if (not visited[neigh_fact]) {
                visited[neigh_fact] = true;
                queue.push(neigh_fact);
            }
        }
    }
    return landmarks;
}

set<int>
find_landmarks_given_N_star_set(LabeledGraph &graph, int start_fact, Strips &problem, vector<bool> &N_star) {
    vector<bool> visited(problem.num_facts, false);
    set<int> landmarks;
    queue<int> queue;
    visited[start_fact] = true;
    queue.push(start_fact);

    while (not queue.empty()) {
        int cur_fact = queue.front();
        queue.pop();

        for (auto& neigh_fact_op_pair : graph.adjacency_out[cur_fact]) {
            int neigh_fact = neigh_fact_op_pair.first;
            int op_idx = neigh_fact_op_pair.second;
            if (N_star[neigh_fact]) {
                landmarks.insert(op_idx);
            } else if (not visited[neigh_fact]) {
                visited[neigh_fact] = true;
                queue.push(neigh_fact);
            }
        }
    }
    return landmarks;
}

vector<int> find_landmarks_backwards_bfs(LabeledGraph &graph, int end_fact, Strips &problem) {
    vector<bool> visited(problem.num_facts, false);
    vector<bool> in_landmarks(problem.operators.size(), false);
    vector<int> landmarks;

    queue<int> queue;
    queue.push(end_fact);
    visited[end_fact] = true;

    while (not queue.empty()) {
        int cur_fact = queue.front();
        queue.pop();

        for (auto& neigh_fact_op_pair : graph.adjacency_in[cur_fact]) {
            int neigh_fact = neigh_fact_op_pair.first;
            int op_idx = neigh_fact_op_pair.second;
            if (problem.operators[op_idx].lm_cost == 0) {
                if (not visited[neigh_fact]) {
                    visited[neigh_fact] = true;
                    queue.push(neigh_fact);
                }
            } else if (not in_landmarks[op_idx] and not visited[neigh_fact]) {
                in_landmarks[op_idx] = true;
                landmarks.emplace_back(op_idx);
            }
        }

    }

    return landmarks;
}

vector<int>
find_landmarks_given_N_star_vect(LabeledGraph &graph, int start_fact, Strips &problem, vector<bool> &N_star) {
    vector<bool> visited(problem.num_facts, false);
    vector<bool> in_landmarks(problem.operators.size(), false);
    vector<int> landmarks;
    queue<int> queue;
    visited[start_fact] = true;
    queue.push(start_fact);

    while (not queue.empty()) {
        int cur_fact = queue.front();
        queue.pop();

        for (auto& neigh_fact_op_pair : graph.adjacency_out[cur_fact]) {
            int neigh_fact = neigh_fact_op_pair.first;
            int op_idx = neigh_fact_op_pair.second;
            if (N_star[neigh_fact]) {
                if ( not in_landmarks[op_idx]) {
                    in_landmarks[op_idx] = true;
                    landmarks.emplace_back(op_idx);
                }
            } else if (not visited[neigh_fact]) {
                visited[neigh_fact] = true;
                queue.push(neigh_fact);
            }
        }
    }
    return landmarks;
}

deque<int>
find_landmarks_given_N_star_deque(LabeledGraph &graph, int start_fact, Strips &problem, vector<bool> &N_star) {
    vector<bool> visited(problem.num_facts, false);
    vector<bool> in_landmarks(problem.operators.size(), false);
    deque<int> landmarks;
    queue<int> queue;
    visited[start_fact] = true;
    queue.push(start_fact);

    while (not queue.empty()) {
        int cur_fact = queue.front();
        queue.pop();

        for (auto& neigh_fact_op_pair : graph.adjacency_out[cur_fact]) {
            int neigh_fact = neigh_fact_op_pair.first;
            int op_idx = neigh_fact_op_pair.second;
            if (N_star[neigh_fact]) {
                if ( not in_landmarks[op_idx]) {
                    in_landmarks[op_idx] = true;
                    landmarks.emplace_back(op_idx);
                }
            } else if (not visited[neigh_fact]) {
                visited[neigh_fact] = true;
                queue.push(neigh_fact);
            }
        }
    }
    return landmarks;
}

vector<bool> find_zero_cost_reachable_from_end(LabeledGraph &graph, int end_fact, const Strips &problem) {
    vector<bool> visited(problem.num_facts, false);
    queue<int> queue;
    queue.push(end_fact);
    visited[end_fact] = true;

    while (not queue.empty()) {
        int cur_fact = queue.front();
        queue.pop();

        for (auto& neigh_fact_op_pair : graph.adjacency_in[cur_fact]) {
            int neigh_fact = neigh_fact_op_pair.first;
            int op_idx = neigh_fact_op_pair.second;
            if (problem.operators[op_idx].lm_cost == 0 and not visited[neigh_fact]) {
                visited[neigh_fact] = true;
                queue.push(neigh_fact);
            }
        }

    }

    return visited;
}

int main(int argc, char *argv[]) {
#ifdef TIME_EXECUTION
    auto start = std::chrono::high_resolution_clock::now();
#endif
    strips_t strips;

    if (argc < 2){
        cerr << "Usage: "<< argv[0] << " problem.strips" << endl;
        return -1;
    }

    stripsRead(&strips, argv[1]);
    Strips myStrips(strips);
    auto final = get_plan_lm(myStrips);
    cout << ";; Cost: " << final.first.cost << endl;
    cout << ";; Init: " << final.second << endl << endl;
    for (auto op_index : final.first.path) {
        cout << strips.operators[op_index].name << endl;
    }

    stripsFree(&strips);

#ifdef TIME_EXECUTION
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    cerr << "Time taken by function: "
         << duration.count() << " microseconds" << endl;
#endif

    return 0;
}
