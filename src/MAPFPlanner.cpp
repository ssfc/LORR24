#include <Entry.h>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/environment.hpp>
#include <Planner/epibt.hpp>
#include <Planner/epibt_lns.hpp>
#include <Planner/pepibt_lns.hpp>
#include <Planner/pibt.hpp>
#include <settings.hpp>

#include <const.h>
#include <planner.h>

/**
 * Initialises the MAPF planner with a given time limit for preprocessing.
 *
 * This function call the planner initialize function with a time limit fore preprocessing.
 *
 * @param preprocess_time_limit The total time limit allocated for preprocessing (in milliseconds).
 */
void MAPFPlanner::initialize(int preprocess_time_limit) {
    int limit = preprocess_time_limit - std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::initialize(limit, env);

    if (get_planner_type() == PlannerType::WPPL) {
        smart_planner.initialize(preprocess_time_limit);
    }

    init_environment(*env);
}

/**
 * Plans a path using default planner
 *
 * This function performs path planning within the timelimit given, and call the plan function in default planner.
 * The planned actions are output to the provided actions vector.
 *
 * @param time_limit The time limit allocated for planning (in milliseconds).
 * @param actions A reference to a vector that will be populated with the planned actions (next action for each agent).
 */
void MAPFPlanner::plan(int time_limit, vector<Action> &actions) {
    ETimer timer;
    TimePoint end_time = env->plan_start_time + Milliseconds(time_limit - 50);
    update_environment(*env);

    if (get_planner_type() == PlannerType::PIBT) {
        PIBT pibt(get_robots_handler().get_robots(), end_time);
        pibt.solve();
        actions = pibt.get_actions();
        PRINT(Printer() << "[PIBT] time: " << timer << '\n';);
    } else if (get_planner_type() == PlannerType::PIBT_TF) {
        int limit = time_limit - std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() - 50;
        DefaultPlanner::plan(limit, actions, env);
        PRINT(Printer() << "[PIBT_TF] time: " << timer << '\n';);
    } else if (get_planner_type() == PlannerType::EPIBT) {
        EPIBT pibt(get_robots_handler().get_robots(), end_time);
        pibt.solve();
        actions = pibt.get_actions();
        PRINT(Printer() << "[EPIBT] solve: " << pibt.get_epibt_steps() << ", time: " << timer << '\n';);
    } else if (get_planner_type() == PlannerType::EPIBT_LNS) {
        EPIBT_LNS pibt(get_robots_handler().get_robots(), end_time);
        pibt.solve(42);
        actions = pibt.get_actions();
        PRINT(Printer() << "[EPIBT_LNS] score: " << pibt.get_score() << '\n';
              Printer() << "[EPIBT_LNS] step:  " << pibt.get_epibt_steps() << "+" << pibt.get_lns_steps() << '\n';
              Printer() << "[EPIBT_LNS] time:  " << timer << '\n';);
    } else if (get_planner_type() == PlannerType::PEPIBT_LNS) {
        PEPIBT_LNS pibt(get_robots_handler().get_robots(), end_time);
        pibt.solve(42);
        actions = pibt.get_actions();
        PRINT(Printer() << "[PEPIBT_LNS] time: " << timer << '\n';);
    } else if (get_planner_type() == PlannerType::WPPL) {
        actions.clear();
        smart_planner.plan(time_limit, actions);
        PRINT(Printer() << "[WPPL] time: " << timer << '\n';);
    } else {
        FAILED_ASSERT("unexpected type");
    }
}
