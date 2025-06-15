#pragma once

#include <unordered_set>
#include <SharedEnv.h>
#include <Tasks.h>

#include <Scheduler/scheduler.hpp>
#include <settings.hpp>

class TaskScheduler {
public:
    SharedEnvironment *env;
    MyScheduler my_scheduler;

    std::unordered_set<int> free_agents;
    std::unordered_set<int> free_tasks;

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

    struct Point {
        int x, y;
    };

    int first_epoch_done_time = -1; // 如果初始阶段的任务较多, 先等它们分配完成. 本变量记录初始分配结束时间。
    std::unordered_map<int, int> task_region; // 记录每个task所属的历史区域

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
    int cross(const Point& startA, const Point& endA, const Point& pointB) {
        return (endA.x - startA.x) * (pointB.y - startA.y) - (endA.y - startA.y) * (pointB.x - startA.x);
    }

    bool isIntersecting(const Point& startA, const Point& endA, const Point& startB, const Point& endB) {
        // Bounding box filter
        if (std::max(startA.x, endA.x) < std::min(startB.x, endB.x) ||
            std::max(startA.y, endA.y) < std::min(startB.y, endB.y) ||
            std::max(startB.x, endB.x) < std::min(startA.x, endA.x) ||
            std::max(startB.y, endB.y) < std::min(startA.y, endA.y)) {
            return false;
        }

        // Step 1: Check general intersection condition
        int cross1 = cross(startA, endA, startB);
        int cross2 = cross(startA, endA, endB);
        int cross3 = cross(startB, endB, startA);
        int cross4 = cross(startB, endB, endA);

        if (cross1 * cross2 < 0 && cross3 * cross4 < 0)
            return true;

        // Step 2: Check collinear overlap
        auto isBetween = [](int a, int b, int c) {
            return std::min(a, b) <= c && c <= std::max(a, b);
        };

        if (cross1 == 0 && isBetween(startA.x, endA.x, startB.x) && isBetween(startA.y, endA.y, startB.y)) return true;
        if (cross2 == 0 && isBetween(startA.x, endA.x, endB.x) && isBetween(startA.y, endA.y, endB.y)) return true;
        if (cross3 == 0 && isBetween(startB.x, endB.x, startA.x) && isBetween(startB.y, endB.y, startA.y)) return true;
        if (cross4 == 0 && isBetween(startB.x, endB.x, endA.x) && isBetween(startB.y, endB.y, endA.y)) return true;

        return false;
    }

    [[nodiscard]] int compute_jam_curr_pickup_intersect_curr_goal(int _agent_id, Point _agent_loc,
                                                                  Point _agent_end);
    void adaptive_jam_curr_pickup_intersect_curr_goal(int time_limit, std::vector<int> & proposed_schedule);

    // 默认分配算法，用于其他分配算法处理首批任务
    void greedy_sum_without_newtask(int time_limit, std::vector<int> & proposed_schedule);

    // 1.2: 将地图分成12x12的区域, task所在区域中agent的数量作为sum jam weight
    void adaptive_jam_task_pickup_region_count_current(int time_limit, std::vector<int> & proposed_schedule);
};
