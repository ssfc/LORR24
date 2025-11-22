#pragma once

#include <Objects/Basic/randomizer.hpp>
#include <Objects/Environment/info.hpp>

#include <settings.hpp>

#include <SharedEnv.h>

#include <cstdint>
#include <vector>

struct SchedulerSolver {

    double cur_score = 0; // 总cost

    // desires[r] = task id
    std::vector<uint32_t> desires;

    // task_to_robot[task] = robot id
    std::vector<uint32_t> task_to_robot;

    std::vector<uint32_t> free_robots;

    std::vector<uint32_t> free_tasks;



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



    std::unordered_map<int, int> task_region; // 记录每个task所属的历史区域
    vector<int> region_agent_num;



    struct Point {
        int x, y;
    };


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





    // dp[r] = отсортированный вектор (dist, task_id)
    std::vector<std::vector<std::pair<uint32_t, uint32_t>>> dp;

    std::vector<int> timestep_updated;

    SharedEnvironment *env = nullptr;

    // task_metric[t]
    std::vector<uint32_t> task_metric;

    // task_target[t] = цель задачи (pos)
    std::vector<uint32_t> task_target;

    double temp = 1;

    void rebuild_dp(uint32_t r);

    [[nodiscard]] bool compare(double cur_score, double old_score, Randomizer &rnd) const;

    template<typename rollback_t>
    bool consider(double old_score, Randomizer &rnd, rollback_t &&rollback) {
        if (compare(cur_score, old_score, rnd)) {
            return true;
        } else {
            rollback();
            ASSERT(std::abs(old_score - cur_score) / std::max(std::abs(old_score), std::abs(cur_score)) < 1e-6,
                   "invalid rollback: " + std::to_string(old_score) + " != " + std::to_string(cur_score) + ", diff: " +
                           std::to_string(old_score - cur_score));
            return false;
        }
    }

    [[nodiscard]] uint64_t get_dist(uint32_t r, uint32_t t);

    [[nodiscard]] uint64_t get_jam_dist(uint32_t r, uint32_t t); // 加入考虑拥堵时间的dist

    void remove(uint32_t r);

    void add(uint32_t r, uint32_t t);

    bool try_peek_task(Randomizer &rnd);

    void validate();

public:
    SchedulerSolver() = default;

    explicit SchedulerSolver(SharedEnvironment *env);

    void update();

    void rebuild_dp(TimePoint end_time);

    void lazy_solve(TimePoint end_time);

    void lns_solve(TimePoint end_time);

    [[nodiscard]] std::vector<int> get_schedule() const;

    [[nodiscard]] double get_score() const;
};
