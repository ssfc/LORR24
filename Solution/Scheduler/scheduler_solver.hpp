#pragma once

#include <Objects/Basic/randomizer.hpp>
#include <settings.hpp>

#include <SharedEnv.h>

#include <cstdint>
#include <vector>

struct SchedulerSolver {

    double cur_score = 0;

    // desires[r] = task id
    // 存储每个机器人当前的目标任务。初始值为 -1，表示机器人没有目标任务。
    std::vector<uint32_t> desires;

    // task_to_robot[task] = robot id
    std::vector<uint32_t> task_to_robot;

    std::vector<uint32_t> free_robots;

    std::vector<uint32_t> free_tasks;

    // dp[r] = отсортированный вектор (dist, task_id)
    // 按升序排列的向量，其中每个元素是键值对 (dist, task_id)，分别表示距离（dist）和任务编号（task_id）。
    std::vector<std::vector<std::pair<uint32_t, uint32_t>>> dp;

    std::vector<int> timestep_updated;

    SharedEnvironment *env = nullptr;

    // task_metric[t]
    std::vector<uint32_t> task_metric; // 每个任务的完成距离

    // task_target[t] = цель задачи (pos)
    std::vector<uint32_t> task_target; // 每个任务的取货点

    double temp = 1;

    // 建立按升序排列的向量dp，其中每个元素是键值对 (dist, task_id)，分别表示距离（dist）和任务编号（task_id）。
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

    [[nodiscard]] uint64_t get_dist(uint32_t r, uint32_t t) const;

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
