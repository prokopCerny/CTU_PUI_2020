#include <iostream>
#include <vector>
#include <set>
#include <unordered_map>
#include <cstdint>
#include <limits>
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

class State;

uint64_t hmax(const State &state, const strips_t &problem, const vector<vector<bool>> &op_preconditions);

vector<bool> init_facts(const strips_t &problem) {
    vector<bool> facts(problem.num_facts, false);
    for (size_t init_idx = 0; init_idx < problem.init_size; init_idx++) {
        facts[problem.init[init_idx]] = true;
    }
    return facts;
}

bool is_goal_facts_subset(const vector<bool> &facts, const strips_t &problem) {
    bool goal = true;
    for (size_t goal_idx = 0; goal_idx < problem.goal_size; goal_idx++) {
        if (not facts[problem.goal[goal_idx]]) {
            goal = false;
            break;
        }
    }
    return goal;
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

    explicit State(const strips_t &problem) :
        cost{0},
        facts( init_facts(problem) ),
        facts_hash{ std::hash<vector<bool>>{}(facts) } {};

    bool is_operator_applicable(const strips_operator_t &op) {
        bool applicable = true;
        for (size_t pre_idx = 0; pre_idx < op.pre_size; pre_idx++) {
            if (not facts[op.pre[pre_idx]]) {
                applicable = false;
                break;
            }
        }
        return applicable;
    }

    bool is_goal_state(const strips_t &problem) {
        return is_goal_facts_subset(facts, problem);
    }

    State apply_operator(const strips_operator_t &op, const int operator_idx) {

        #ifdef MY_DEBUG
        if(! is_operator_applicable(op)) {
            cerr << "Can't apply operator!!!" << endl;
            exit(-1);
        }
        #endif

        vector<bool> new_facts(facts);
        vector<int> new_path(path);

        uint64_t new_cost = cost + op.cost;
        new_path.push_back(operator_idx);

        for (size_t del_idx = 0; del_idx < op.del_eff_size; del_idx++) {
            new_facts[op.del_eff[del_idx]] = false;
        }

        for (size_t add_idx = 0; add_idx < op.add_eff_size; add_idx++) {
            new_facts[op.add_eff[add_idx]] = true;
        }
        State succ(new_facts, new_path, new_cost);
        return succ;
    }
};

struct my_less_compare {
    bool operator() (const std::pair<uint64_t, State> &lhs, const std::pair<uint64_t, State> &rhs) {
        if (lhs.first < rhs.first) {
            return true;
        } else if ( (lhs.first == rhs.first) and (lhs.second.facts_hash < rhs.second.facts_hash) ) {
            return true;
        } else {
            return false;
        }
    }
};

/**
 * A* using pairs of <cost, State object>
 * @param problem
 * @return
 */
State get_plan_1(strips_t &problem, const vector<vector<bool>> &op_preconditions) {
    unordered_map<size_t, uint64_t> distance;
    set<std::pair<uint64_t, State>, my_less_compare> queue;

    State start_state(problem);
    distance[start_state.facts_hash] = 0;
    queue.insert(std::make_pair(0, std::move(start_state)));

    while (not queue.empty()) {
        std::pair<uint64_t, State> cur_state_pair = *(queue.begin());
        queue.erase(queue.begin());

        uint64_t cost = cur_state_pair.first;
        State cur_state = std::move(cur_state_pair.second);

        if (cur_state.is_goal_state(problem)) {
            return cur_state;
        }

        for (size_t op_idx = 0; op_idx < problem.num_operators; op_idx++) {
            strips_operator_t &current_operator = problem.operators[op_idx];

            if (cur_state.is_operator_applicable(current_operator)) {
                State next_state = cur_state.apply_operator(current_operator, op_idx);
                uint64_t heuristic_val = 0; //TODO
                uint64_t predicted_cost = next_state.cost + heuristic_val;
                size_t appearance_count = distance.count(next_state.facts_hash);

                if (appearance_count == 0 || distance[next_state.facts_hash] > predicted_cost) {
                    auto temp_pair = std::make_pair(predicted_cost, std::move(next_state));
                    if (appearance_count != 0) {
                        queue.erase(queue.find(temp_pair));
                    }

                    distance[temp_pair.second.facts_hash] = predicted_cost;
                    queue.insert(std::move(temp_pair));
                }
            }
        }

    }

    State not_found = State(problem);
    return not_found;
}

/**
 * A* using pairs of <cost, hash> and a supplementary map of <hash, State object>
 * @param problem
 * @return
 */
State get_plan_2(const strips_t &problem, const vector<vector<bool>> &op_preconditions) {
    unordered_map<size_t, uint64_t> distance;
    unordered_map<size_t, State> states;
    set<std::pair<uint64_t, size_t>> queue;

    State start_state(problem);
    distance[start_state.facts_hash] = 0;
    queue.insert(std::make_pair(0, start_state.facts_hash));
    states.insert(std::make_pair(start_state.facts_hash, std::move(start_state)));

    while (not queue.empty()) {
        State cur_state = std::move(states.find(queue.begin()->second)->second);
        states.erase(cur_state.facts_hash);
        queue.erase(queue.begin());

        if (cur_state.is_goal_state(problem)) {
            return cur_state;
        }

        for (size_t op_idx = 0; op_idx < problem.num_operators; op_idx++) {
            strips_operator_t &current_operator = problem.operators[op_idx];

            if (cur_state.is_operator_applicable(current_operator)) {
                State next_state = cur_state.apply_operator(current_operator, op_idx);
                uint64_t heuristic_val = hmax(next_state, problem, op_preconditions); //TODO
                uint64_t predicted_cost = next_state.cost + heuristic_val;
                size_t appearance_count = distance.count(next_state.facts_hash);

                if (appearance_count == 0 || distance[next_state.facts_hash] > predicted_cost) {
                    if (appearance_count != 0) {
                        queue.erase(queue.find(std::make_pair(distance[next_state.facts_hash], next_state.facts_hash)));
                    }

                    distance[next_state.facts_hash] = predicted_cost;
                    queue.insert(std::make_pair(predicted_cost, next_state.facts_hash));

                    auto it = states.find(next_state.facts_hash);
                    if ( it != states.end() ) {
                        it->second = std::move(next_state);
                    } else {
                        states.insert(std::make_pair(next_state.facts_hash, std::move(next_state)));
                    }
                }
            }
        }

    }

    State not_found = State(problem);
    return not_found;
}

vector<vector<bool>> create_op_preconditions(const strips_t &problem) {
    vector<vector<bool>> op_preconditions(problem.num_operators, vector<bool>(problem.num_facts, false));
    for (size_t op_idx = 0; op_idx < problem.num_operators; op_idx++) {
        strips_operator_t &current_operator = problem.operators[op_idx];
        for (size_t pre_idx = 0; pre_idx < current_operator.pre_size; pre_idx++) {
            op_preconditions[op_idx][current_operator.pre[pre_idx]] = true;
        }
    }
    return op_preconditions;
}

uint64_t hmax(const State &state, const strips_t &problem, const vector<vector<bool>> &op_preconditions) {
    static unordered_map<size_t, uint64_t> cache;

    if (cache.count(state.facts_hash) != 0) {
        return cache[state.facts_hash];
    }

    const vector<bool> &state_facts = state.facts;
    vector<int> delta(problem.num_facts, std::numeric_limits<int>::max());
    for (size_t fact_idx = 0; fact_idx < problem.num_facts; ++fact_idx) {
        if (state_facts[fact_idx]) {
            delta[fact_idx] = 0;
        }
    }

    vector<int> U(problem.num_operators, -1);

    for (size_t op_idx = 0; op_idx < problem.num_operators; ++op_idx) {
        strips_operator_t &current_operator = problem.operators[op_idx];

        U[op_idx] = current_operator.pre_size;

        if (current_operator.pre_size == 0) {
            for (size_t add_idx = 0; add_idx < current_operator.add_eff_size; add_idx++) {
                size_t current_add_fact = current_operator.add_eff[add_idx];
                delta[current_add_fact] = std::min(delta[current_add_fact], current_operator.cost);
            }
        }
    }


    vector<bool> C(problem.num_facts, false);

    set<std::pair<int, size_t>> queue;
    for (size_t fact_idx = 0; fact_idx < problem.num_facts; fact_idx++) {
        queue.emplace(delta[fact_idx], fact_idx);
    }

    while( not is_goal_facts_subset(C, problem)) {
        size_t k = queue.begin()->second;
        queue.erase(queue.begin());
        C[k] = true;

        for (size_t op_idx = 0; op_idx < problem.num_operators; ++op_idx) {
            if (op_preconditions[op_idx][k]) {
                strips_operator_t &current_operator = problem.operators[op_idx];
                U[op_idx]--;

                if (U[op_idx] == 0) {
                    for (size_t add_idx = 0; add_idx < current_operator.add_eff_size; add_idx++) {
                        int cur_add_fact = current_operator.add_eff[add_idx];
                        int potentialCost = current_operator.cost + delta[k];

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

    uint64_t heur{0};
    for (size_t goal_idx = 0; goal_idx < problem.goal_size; goal_idx++) {
        heur = std::max(heur, (uint64_t) delta[problem.goal[goal_idx]]);
    }

    cache[state.facts_hash] = heur;
    return heur;
}


int main(int argc, char *argv[]) {
#ifdef TIME_EXECUTION
    auto start = std::chrono::high_resolution_clock::now();
#endif
    strips_t strips;

    if (argc != 3){
        cerr << "Usage: "<< argv[0] << " problem.strips problem.fdr" << endl;
        return -1;
    }

    stripsRead(&strips, argv[1]);
    State start_state(strips);
    const vector<vector<bool>> op_preconditions = create_op_preconditions(strips);
    State final = get_plan_2(strips, op_preconditions);
    cout << ";; Cost: " << final.cost << endl;
    cout << ";; Init: " << hmax(start_state, strips, op_preconditions) << endl << endl;
    for (auto op_index : final.path) {
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
