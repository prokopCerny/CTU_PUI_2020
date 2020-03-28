#include <iostream>
#include <vector>
#include <set>
#include <cstdint>
#include "problem.h"

using std::cout;
using std::cerr;
using std::endl;
using std::vector;
using std::set;



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
        facts_hash{ std::hash<vector<bool>>{}(succ_facts) } {}

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

int main(int argc, char *argv[]) {
    strips_t strips;

    if (argc != 3){
        cerr << "Usage: "<< argv[0] << " problem.strips problem.fdr" << endl;
        return -1;
    }

    stripsRead(&strips, argv[1]);
    for (size_t i = 0; i < strips.num_facts; i++) {
        cout << strips.fact_names[i] << endl;
    }

    stripsFree(&strips);
    return 0;
}
