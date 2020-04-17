#include <iostream>
#include <vector>
#include <set>
#include <unordered_map>
#include <cstdint>
#include <limits>
#include <string>
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

class State;

class StripsOperator {
public:
    string name;
    int cost;
    vector<int> preconditions;
    vector<int> add_effects;
    vector<int> del_effects;

    explicit StripsOperator(strips_operator_t &op) :
        name(op.name),
        cost{op.cost},
        preconditions(op.pre, op.pre + op.pre_size),
        add_effects(op.add_eff, op.add_eff + op.add_eff_size),
        del_effects(op.del_eff, op.del_eff + op.del_eff_size) {}
};

class Strips {
public:
    int num_facts;
    vector<string> fact_names;
    vector<int> initial_state;
    vector<int> goal_state;
    vector<StripsOperator> operators;

    explicit Strips(strips_t &strips) :
        num_facts{strips.num_facts},
        fact_names(strips.fact_names, strips.fact_names + num_facts),
        initial_state(strips.init, strips.init + strips.init_size),
        goal_state(strips.goal, strips.goal + strips.goal_size),
        operators(strips.operators, strips.operators + strips.num_operators) {}
};

uint64_t hmax(const State &state, const strips_t &problem, const vector<vector<bool>> &op_preconditions);

vector<bool> init_facts(const strips_t &problem) {
    vector<bool> facts(problem.num_facts, false);
    for (size_t init_idx = 0; init_idx < problem.init_size; init_idx++) {
        facts[problem.init[init_idx]] = true;
    }
    return facts;
}

vector<bool> init_facts(const Strips &problem) {
    vector<bool> facts(problem.num_facts, false);
    for (auto fact : problem.initial_state) {
        facts[fact] = true;
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

bool is_goal_facts_subset(const vector<bool> &facts, const Strips &problem) {
    bool goal = true;
    for (auto goal_fact : problem.goal_state) {
        if (not facts[goal_fact]) {
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

    explicit State(const Strips &problem) :
            cost{0},
            facts( init_facts(problem) ),
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

    bool is_goal_state(const strips_t &problem) const {
        return is_goal_facts_subset(facts, problem);
    }

    bool is_goal_state(const Strips &problem) const {
        return is_goal_facts_subset(facts, problem);
    }

    State apply_operator(const strips_operator_t &op, const int operator_idx) const {

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

    State apply_operator(const StripsOperator &op, const int operator_idx) {

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

        for (auto del_fact : op.del_effects) {
            new_facts[del_fact] = false;
        }

        for (auto add_fact : op.add_effects) {
            new_facts[add_fact] = true;
        }

        State succ(new_facts, new_path, new_cost);
        return succ;
    }
};

/**
 * A* using pairs of <cost, hash> and a supplementary map of <hash, State object>
 * @param problem
 * @return
 */
State get_plan(const strips_t &problem, const vector<vector<bool>> &op_preconditions) {
    unordered_map<size_t, uint64_t> distance;
    unordered_map<size_t, State> states;
    set<std::pair<uint64_t, size_t>> queue;

    State start_state(problem);
    distance[start_state.facts_hash] = 0;
    queue.insert(std::make_pair(0, start_state.facts_hash));
    states.insert(std::make_pair(start_state.facts_hash, std::move(start_state)));

    while (not queue.empty()) {
        const State cur_state = std::move(states.find(queue.begin()->second)->second);
        states.erase(cur_state.facts_hash);
        queue.erase(queue.begin());

        if (cur_state.is_goal_state(problem)) {
            return cur_state;
        }

        for (size_t op_idx = 0; op_idx < problem.num_operators; op_idx++) {
            const strips_operator_t &current_operator = problem.operators[op_idx];

            if (cur_state.is_operator_applicable(current_operator)) {
                State next_state = cur_state.apply_operator(current_operator, op_idx);
                uint64_t heuristic_val = hmax(next_state, problem, op_preconditions); //TODO
                uint64_t predicted_cost = next_state.cost + heuristic_val;
                if (predicted_cost < heuristic_val) {
                    predicted_cost = std::numeric_limits<uint64_t>::max();
                }
                auto appearance_it = distance.find(next_state.facts_hash);

                uint64_t cur_cost = distance[next_state.facts_hash];
                if (appearance_it == distance.end() || cur_cost > predicted_cost) {
                    if (appearance_it != distance.end()) {
                        queue.erase(queue.find(std::make_pair(distance[next_state.facts_hash], next_state.facts_hash)));
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
    typedef uint32_t FACT_COST_TYPE;

    static unordered_map<size_t, uint64_t> cache;

    if (cache.count(state.facts_hash) != 0) {
        return cache[state.facts_hash];
    }

    const vector<bool> &state_facts = state.facts;
    vector<FACT_COST_TYPE> delta(problem.num_facts, std::numeric_limits<FACT_COST_TYPE>::max());
//    vector<unsigned int> delta(problem.num_facts, std::numeric_limits<unsigned int>::max());
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
                delta[current_add_fact] = std::min(delta[current_add_fact], (FACT_COST_TYPE) current_operator.cost);
            }
        }
    }


    vector<bool> C(problem.num_facts, false);

//    std::unordered_map<size_t, FACT_COST_TYPE> fact_costs;

    set<std::pair<FACT_COST_TYPE, size_t>> queue;
    for (size_t fact_idx = 0; fact_idx < problem.num_facts; fact_idx++) {
        queue.emplace(delta[fact_idx], fact_idx);
//        fact_costs.emplace(fact_idx, delta[fact_idx]);
    }

    while( not is_goal_facts_subset(C, problem)) {
//        size_t k{0};
//        uint64_t min_val{std::numeric_limits<FACT_COST_TYPE>::max() + 1ull};
//        for (auto &it : fact_costs) {
//            if (it.second < min_val) {
//                k = it.first;
//                min_val = it.second;
//            }
//        }
        size_t k = queue.begin()->second;
        queue.erase(queue.begin());
        C[k] = true;
//        fact_costs.erase(fact_costs.find(k));

        for (size_t op_idx = 0; op_idx < problem.num_operators; ++op_idx) {
            if (op_preconditions[op_idx][k]) {
                strips_operator_t &current_operator = problem.operators[op_idx];
                U[op_idx]--;

                if (U[op_idx] == 0) {
                    for (size_t add_idx = 0; add_idx < current_operator.add_eff_size; add_idx++) {
                        int cur_add_fact = current_operator.add_eff[add_idx];
                        FACT_COST_TYPE potentialCost = (FACT_COST_TYPE) current_operator.cost + delta[k];
                        if (potentialCost < delta[k]) {
                            potentialCost = std::numeric_limits<FACT_COST_TYPE>::max();
                        }

                        if (delta[cur_add_fact] > potentialCost) {
                            if (not C[cur_add_fact]) {
                                queue.erase(queue.find(std::make_pair(delta[cur_add_fact], cur_add_fact)));
                                queue.emplace(potentialCost, cur_add_fact);
//                                fact_costs[cur_add_fact] = potentialCost;
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

    if (argc < 2){
        cerr << "Usage: "<< argv[0] << " problem.strips" << endl;
        return -1;
    }

    stripsRead(&strips, argv[1]);
//    Strips myStrips(strips);
    State start_state(strips);
    const vector<vector<bool>> op_preconditions = create_op_preconditions(strips);
    State final = get_plan(strips, op_preconditions);
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
