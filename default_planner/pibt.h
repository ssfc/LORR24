
#ifndef pibt_hpp
#define pibt_hpp

#include "TrajLNS.h"
#include "Types.h"
#include "heuristics.h"
#include "utils.h"
#include <list>
#include <tuple>
#include <unordered_set>
#include <vector>


namespace DefaultPlanner {

    int get_gp_h(TrajLNS &lns, int ai, int target);


    bool causalPIBT(int curr_id, int higher_id, std::vector<State> &prev_states,
                    std::vector<State> &next_states,
                    std::vector<int> &prev_decision, std::vector<int> &decision,
                    std::vector<bool> &occupied, TrajLNS &lns);


    Action getAction(State &prev, State &next);

    Action getAction(State &prev, int next_loc, SharedEnvironment *env);

    bool moveCheck(int id, std::vector<bool> &checked,
                   std::vector<DCR> &decided, std::vector<Action> &actions, std::vector<int> &prev_decision);
}// namespace DefaultPlanner
#endif