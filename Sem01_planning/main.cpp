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

class State;

typedef int32_t FACT_COST_TYPE;

class LabeledGraph {
public:
    size_t vertices;
    vector<list<pair<size_t, int>>> adjacency_out;
    vector<list<pair<size_t, int>>> adjacency_in;

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
    int support{-1};
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

    void reset_lm_cost_and_support() {
        lm_cost = cost;
        support = -1;
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
            lm_goal.push_back(num_facts++);
            fact_names.emplace_back("LM-Goal-fact");
            lm_init.push_back(num_facts++);
            fact_names.emplace_back("LM-Init-fact");

            operators.emplace_back("new_goal_op", 0);
            StripsOperator &lm_goal_op = operators.back();
            lm_goal_op.add_effects.push_back(lm_goal.front());
            lm_goal_op.preconditions.insert(lm_goal_op.preconditions.end(), goal_state.begin(), goal_state.end());

            operators.emplace_back("new_init_op", 0);
            StripsOperator &lm_init_op = operators.back();
            lm_init_op.preconditions.push_back(lm_init.front());
        }
    }
};

uint64_t lmcut(const State &state,
               Strips &problem,
               const vector<vector<bool>> &op_preconditions,
               const vector<bool> &original_init_facts,
               const vector<bool> &lm_init_facts);

vector<vector<bool>> create_op_preconditions(const Strips &problem);

uint64_t hmax(const State &state, const strips_t &problem, const vector<vector<bool>> &op_preconditions);

vector<bool> find_zero_cost_reachable_from_end(LabeledGraph &graph, int end_fact, const Strips &problem);

unordered_set<int>
find_landmarks_given_N_star(LabeledGraph &graph, int start_fact, Strips &problem, vector<bool> &N_star);

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

vector<bool> init_facts(size_t num_facts, const vector<int> &facts_lst) {
    vector<bool> facts(num_facts, false);
    for (auto fact : facts_lst) {
        facts[fact] = true;
    }
    return facts;
}

bool is_goal_facts_subset(const vector<bool> &facts, const strips_t &problem) {
    bool goal = true;
    for (int goal_idx = 0; goal_idx < problem.goal_size; goal_idx++) {
        if (not facts[problem.goal[goal_idx]]) {
            goal = false;
            break;
        }
    }
    return goal;
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

    explicit State(const strips_t &problem) :
        facts( init_facts(problem) ),
        cost{0},
        facts_hash{ std::hash<vector<bool>>{}(facts) } {};

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

    while (not queue.empty()) {
        //uint64_t cur_val = queue.begin()->first;
        State cur_state = std::move(states.find(queue.begin()->second)->second);
        states.erase(cur_state.facts_hash);
        queue.erase(queue.begin());

        if (cur_state.is_goal_state(problem)) {
            return std::make_pair(std::move(cur_state), initial_lmcut);
        }

        for (size_t op_idx = 0; op_idx < problem.operators.size(); op_idx++) {
            StripsOperator &current_operator = problem.operators[op_idx];

            if (cur_state.is_operator_applicable(current_operator)) {
                State next_state = cur_state.apply_operator(current_operator, op_idx);
                uint64_t heuristic_val = lmcut(next_state, problem, op_preconditions, original_init_facts, lm_init_facts); //TODO
                uint64_t predicted_cost = next_state.cost + heuristic_val;
                if (predicted_cost < heuristic_val) {
                    predicted_cost = std::numeric_limits<uint64_t>::max();
                }
                auto appearance_it = distance.find(next_state.facts_hash);

                uint64_t cur_cost = distance[next_state.facts_hash];
                if (appearance_it == distance.end() || cur_cost > predicted_cost) {
                    if (appearance_it != distance.end()) {
                        auto it = queue.find(std::make_pair(distance[next_state.facts_hash], next_state.facts_hash));
                        if (it != queue.end()) {
                            queue.erase(it);
                            distance[next_state.facts_hash] = predicted_cost;
                            queue.emplace(predicted_cost, next_state.facts_hash);

                            auto it2 = states.find(next_state.facts_hash);
                            if ( it2 != states.end() ) {
                                it2->second = std::move(next_state);
                            } else {
                                states.emplace(next_state.facts_hash, std::move(next_state));
                            }
                        }
                    } else {
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
    for (int goal_idx = 0; goal_idx < problem.goal_size; goal_idx++) {
        heur = std::max(heur, (uint64_t) delta[problem.goal[goal_idx]]);
    }

    cache[state.facts_hash] = heur;
    return heur;
}

// ########################################################## ALTERNATIVE HMAX #####################################################################################
int find_min_dist(vector<int>& distances, vector<bool>& C){
    int min = std::numeric_limits<int>::max();
    int min_idx = -1;
    for (int i = 0; i < distances.size(); i++){
        if (!C[i]) {
            if (distances[i] <= min) {
                min = distances[i];
                min_idx = i;
            }
        }
    }
    return min_idx;
}

pair<int, vector<int>> computeHmaxHeuristic(Strips& strips, const vector<bool>& state_facts, const vector<int> &goal,  const vector<vector<bool>> &op_preconditions){
    vector<int> distances(strips.num_facts, std::numeric_limits<int>::max());
    for (int i = 0; i < strips.num_facts; i++) {
        if (state_facts[i]) {
            distances[i] = 0;
        }
    }
    vector<int> U(strips.operators.size(), -1);

    for (int oper_idx = 0; oper_idx < strips.operators.size(); oper_idx++){
        StripsOperator& oper = strips.operators[oper_idx];
        U[oper_idx] = oper.preconditions.size();
        if (oper.preconditions.empty()) {
            for (int add_fact_idx = 0; add_fact_idx < oper.add_effects.size(); add_fact_idx++){
                int fact_idx = oper.add_effects[add_fact_idx];
                distances[fact_idx] = std::min(oper.lm_cost, distances[fact_idx]);
            }
        }
    }

    vector<bool> C(strips.num_facts, false);
//    while(not isGoal(C, goal)){
    while(not is_goal_facts_subset(C, goal)){
        int k = find_min_dist(distances, C);
        C[k] = true;
        for (unsigned int oper_idx = 0; oper_idx < strips.operators.size(); oper_idx++) {
            StripsOperator& oper = strips.operators[oper_idx];
//            if (factInPreconds(k, oper)) {
            if (op_preconditions[oper_idx][k]) {
                U[oper_idx]--;
                if (U[oper_idx] == 0){
                    for (int add_fact_idx = 0; add_fact_idx < oper.add_effects.size(); add_fact_idx++){
                        int fact_idx = oper.add_effects[add_fact_idx];
                        int potential_cost = distances[k] + oper.lm_cost;
                        if (potential_cost < 0) {
                            potential_cost = std::numeric_limits<int>::max();
                        }
                        distances[fact_idx] = std::min(distances[fact_idx], potential_cost);
                    }
                }
            }
        }
    }
    int my_max = std::numeric_limits<int>::min();
    for (int goal_fact_idx = 0; goal_fact_idx < goal.size(); goal_fact_idx++) {
        int fact_idx = goal[goal_fact_idx];
        my_max = std::max(my_max, distances[fact_idx]);
    }

//    heuristics_map[state.key] = my_max;
    return std::make_pair(my_max, std::move(distances));
}

// ########################################################## END ALTERNATIVE #####################################################################################

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

        U[op_idx] = current_operator.preconditions.size();

        if (current_operator.preconditions.empty()) {
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
        anOperator.reset_lm_cost_and_support();
    }

    if (not orig_init_hmax_computed) {
        orig_init_hmax_computed = true;
        pair<int, vector<int>> orig_hmax = hmax_for_lm(original_init_facts, problem, problem.goal_state, op_preconditions);
        orig_init_hmax = orig_hmax.first;
    }

//    pair<int, vector<int>> orig_hmax = hmax_for_lm(original_init_facts, problem, problem.goal_state, op_preconditions);
//    pair<int, vector<int>> orig_hmax = hmax_for_lm(original_init_facts, problem, problem.goal_state, op_preconditions);
//    pair<int, vector<int>> orig_hmax = computeHmaxHeuristic(problem, original_init_facts, problem.goal_state, op_preconditions);
    if (orig_init_hmax == std::numeric_limits<FACT_COST_TYPE>::max()) {
        cache[state.facts_hash] = std::numeric_limits<FACT_COST_TYPE>::max();
        return std::numeric_limits<FACT_COST_TYPE>::max();
    }

    StripsOperator &init_op = problem.operators.back();
    init_op.add_effects.clear();
    for (int fact_idx = 0; fact_idx < problem.num_facts; fact_idx++) {
        if (state.facts[fact_idx]) {
            init_op.add_effects.push_back(fact_idx);
        }
    }

    uint64_t h_lmcut{0};

//    uint64_t hmax_val{0};
//    vector<FACT_COST_TYPE> delta;
    int hmax_val{};
    vector<int> delta;
    std::tie(hmax_val, delta) = hmax_for_lm(lm_init_facts, problem, problem.lm_goal, op_preconditions);
//    std::tie(hmax_val, delta) = computeHmaxHeuristic(problem, lm_init_facts, problem.lm_goal, op_preconditions);
    while (hmax_val > 0 and hmax_val < std::numeric_limits<int>::max()) {
//        cout << h_lmcut << " " << hmax_val << endl;
        LabeledGraph justification(problem.num_facts);
        for (int op_idx = 0; op_idx < problem.operators.size(); op_idx++) {
            StripsOperator &op = problem.operators[op_idx];
            FACT_COST_TYPE max_cost = std::numeric_limits<FACT_COST_TYPE>::min();
            for (int pre : op.preconditions) {
                if (delta[pre] > max_cost) {
                    max_cost = delta[pre];
                    op.support = pre;
                }
            }
            if (op.support >= 0) {
                for (int add_fact : op.add_effects) {
                    justification.add_edge(op.support, add_fact, op_idx);
                }
            }
        }

        vector<bool> N_star = find_zero_cost_reachable_from_end(justification, problem.num_facts-2, problem);
        auto landmarks = find_landmarks_given_N_star(justification, problem.num_facts - 1, problem, N_star);

        int min_landmark_cost{std::numeric_limits<int>::max()};
        for (int landmark : landmarks) {
            StripsOperator &landmarkOp = problem.operators[landmark];
            if (landmarkOp.lm_cost < min_landmark_cost) {
                min_landmark_cost = landmarkOp.lm_cost;
            }
        }


//        uint64_t prev_h_lmcut{h_lmcut};
        h_lmcut += min_landmark_cost;
//        if (h_lmcut < prev_h_lmcut) {
//            h_lmcut = std::numeric_limits<uint64_t>::max();
//            break;
//        }
//        if (min_landmark_cost < 0) {
//            cout << "wtf" << endl;
//        }
        for (int landmark: landmarks) {
            problem.operators[landmark].lm_cost -= min_landmark_cost;
            if (problem.operators[landmark].lm_cost < 0) {
                cout << "wtf2" << endl;
                problem.operators[landmark].lm_cost = 0;
            }
        }

        std::tie(hmax_val, delta) = hmax_for_lm(lm_init_facts, problem, problem.lm_goal, op_preconditions);
//        std::tie(hmax_val, delta) = computeHmaxHeuristic(problem, lm_init_facts, problem.lm_goal, op_preconditions);
        ;
    }

    if (hmax_val >= std::numeric_limits<int>::max() or hmax_val < 0) {
        h_lmcut = std::numeric_limits<int>::max();
    }
//    cout << "leave" << endl;
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
