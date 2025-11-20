#pragma once

#include <Scheduler/scheduler_solver.hpp>

#include <SharedEnv.h>
#include <Tasks.h>

struct MyScheduler {

    SharedEnvironment *env = nullptr;

    SchedulerSolver solver;

    std::vector<int> prev_schedule; // 用来记录proposed_schedule的变化情况

    /*
    struct AgentTask
    {
        int task_id = -1;
        int min_task_dist = -1;
        double jam_when_assign = -1; // 当该任务分配时, agent前往该任务的拥堵系数
        double task_heuristic = -1; // 启发值必然是double, 因为即使sum jam weight是int, 乘以系数后还是要变成double
        int dist_agent_pickup = -1; // 从agent当前位置到pickup点的启发式距离
        int dist_pickup_delivery = -1; // task pickup点到delivery点的启发式距离
        int assign_moment = -1;
        int complete_moment = -1;
    };
    vector<AgentTask> agent_task;

    int numTaskFinished = 0;
    int total_min_span = 0; // 已完成任务的理论完成时间下界之和
    int total_real_duration = 0; // 已完成任务的实际完成时间之和
    double total_jam = 0;
    double jam_coefficient = 1; // 在多大程度上考虑拥堵
    */





    MyScheduler() = default;

    explicit MyScheduler(SharedEnvironment *env);

    void plan(TimePoint end_time, std::vector<int> &proposed_schedule);

    void solver_schedule(TimePoint end_time, std::vector<int> &proposed_schedule);

    void hungarian_schedule(TimePoint end_time, std::vector<int> &proposed_schedule);
};
