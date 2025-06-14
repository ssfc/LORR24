#pragma once

#include <SharedEnv.h>
#include <Tasks.h>

#include <Scheduler/scheduler.hpp>
#include <settings.hpp>

class TaskScheduler {
public:
    SharedEnvironment *env;
    MyScheduler my_scheduler;

    struct Point {
        int x, y;
    };

    explicit TaskScheduler(SharedEnvironment *env) : env(env), my_scheduler(env) {

    }

    TaskScheduler() {
        env = new SharedEnvironment();
    }

    virtual ~TaskScheduler() {
        delete env;
    }

    virtual void initialize(int preprocess_time_limit);

    virtual void plan(int time_limit, std::vector<int> &proposed_schedule);

    // 11: compute pickup jam by counting whether other agent-task line intersect with this agent-task line
    [[nodiscard]] int compute_jam_curr_pickup_intersect_curr_goal(int _agent_id, Point _agent_loc,
                                                                  Point _agent_end);
    void adaptive_jam_curr_pickup_intersect_curr_goal(int time_limit, std::vector<int> & proposed_schedule);


};
