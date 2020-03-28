#include <iostream>
#include <vector>
#include <set>
#include <unordered_map>
#include <cstdint>
#include "problem.h"

using std::cout;
using std::cerr;
using std::endl;
using std::vector;
using std::set;
using std::unordered_map;



vector<bool> init_facts(strips_t &problem) {
    vector<bool> facts(problem.num_facts, false);
    for (size_t init_idx = 0; init_idx < problem.init_size; init_idx++) {
        facts[problem.init[init_idx]] = true;
    }
    return facts;
}

class State {
private:
    State(vector<bool> succ_facts, vector<int> succ_path, uint64_t new_cost) :
        facts(std::move(succ_facts)),
        path(std::move(succ_path)),
        cost{new_cost},
        facts_hash{ std::hash<vector<bool>>{}(facts) } {}

public:
    const vector<bool> facts;
    const vector<int> path;
    const uint64_t cost;
    const size_t facts_hash;

    explicit State(strips_t &problem) :
        cost{0},
        facts( init_facts(problem) ),
        facts_hash{ std::hash<vector<bool>>{}(init_facts(problem)) } {};

    bool is_operator_applicable(strips_operator_t &op) {
        bool applicable = true;
        for (size_t pre_idx = 0; pre_idx < op.pre_size; pre_idx++) {
            if (! facts[op.pre[pre_idx]]) {
                applicable = false;
                break;
            }
        }
        return applicable;
    }

    bool is_goal_state(strips_t &problem) {
        bool goal = true;
        for (size_t goal_idx = 0; goal_idx < problem.goal_size; goal_idx++) {
            if (not facts[problem.goal[goal_idx]]) {
                goal = false;
                break;
            }
        }
        return goal;
    }

    State apply_operator(strips_operator_t &op, int operator_idx) {

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
State get_plan(strips_t &problem) {
    unordered_map<size_t, uint64_t> distance;
    set<std::pair<uint64_t, State>, my_less_compare> queue;

    State start_state(problem);
    distance[start_state.facts_hash] = 0;
    queue.insert(std::make_pair(0, std::move(start_state)));

    while (not queue.empty()) {
        std::pair<uint64_t, State> cur_state_pair(*(queue.begin()));

        queue.erase(queue.begin());

        uint64_t cost = cur_state_pair.first;
        State cur_state = cur_state_pair.second;

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
                    if (appearance_count != 0) {
                        queue.erase(queue.find(std::make_pair(distance[next_state.facts_hash], next_state)));
                    }

                    distance[next_state.facts_hash] = predicted_cost;
                    queue.insert(std::make_pair(predicted_cost, std::move(next_state)));
                }
            }
        }

    }

    State not_found = State(problem);
    return not_found;
}

int main(int argc, char *argv[]) {
    strips_t strips;

    if (argc != 3){
        cerr << "Usage: "<< argv[0] << " problem.strips problem.fdr" << endl;
        return -1;
    }

    stripsRead(&strips, argv[1]);

    State final = get_plan(strips);
    for (auto op_index : final.path) {
        cout << strips.operators[op_index].name << endl;
    }

    stripsFree(&strips);
    return 0;
}
