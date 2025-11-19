#pragma once

#include <Scheduler/scheduler_solver.hpp>

#include <SharedEnv.h>
#include <Tasks.h>

struct MyScheduler {

    SharedEnvironment *env = nullptr;

    SchedulerSolver solver;

    std::vector<int> prev_schedule; // 用来记录proposed_schedule的变化情况


    MyScheduler() = default;

    explicit MyScheduler(SharedEnvironment *env);

    void plan(TimePoint end_time, std::vector<int> &proposed_schedule);

    void solver_schedule(TimePoint end_time, std::vector<int> &proposed_schedule);

    void hungarian_schedule(TimePoint end_time, std::vector<int> &proposed_schedule);
};
