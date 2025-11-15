#include <Scheduler/scheduler_solver.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Basic/time.hpp>
#include <Objects/Environment/environment.hpp>
#include <Tools/tools.hpp>

#include <atomic>
#include <unordered_set>

// 为单个机器人 r 构建 dp[r]。
// dp[r] 是一个 候选任务列表，每个元素是 (距离, task_id) 的 pair。也就是：机器人 r 到每个可选任务之间的成本估计列表。
void SchedulerSolver::rebuild_dp(uint32_t r) {
    dp[r].clear(); // 清空机器人 r 原来的 dp 列表（因为任务变化或时间变化，之前的计算失效）。
    // 遍历当前所有 未被分配（free）的任务 t
    for (uint32_t t: free_tasks) {
        // 计算机器人 r 到任务 t 的距离（或代价）：get_dist(r, t)  将 (距离, t) 这个 pair 添加到 dp[r]
        dp[r].emplace_back(get_dist(r, t), t);
    }
    std::sort(dp[r].begin(), dp[r].end()); // 排序后，dp[r][0] 就是 r 最适合执行的任务（最短距离/最便宜）。
    timestep_updated[r] = env->curr_timestep + 1;
}

// 为多个机器人并行重建 dp 列表的函数。
void SchedulerSolver::rebuild_dp(TimePoint end_time) {
    ETimer timer;
    std::vector<uint32_t> order = free_robots; // 当前空闲机器人列表 free_robots。
    // 根据 timestep_updated 排序：优先更新 dp 时间最久的机器人。
    std::stable_sort(order.begin(), order.end(), [&](uint32_t lhs, uint32_t rhs) {
        return timestep_updated[lhs] < timestep_updated[rhs];
    });

    // 多线程更新
    std::atomic<uint32_t> counter{};
    launch_threads(THREADS, [&](uint32_t thr) {
        for (uint32_t i = thr; i < order.size() && get_now() < end_time; i += THREADS) {
            rebuild_dp(order[i]);
            ++counter;
        }
    });

    PRINT(
            uint32_t p = counter * 100 / order.size();
            ASSERT(0 <= p && p <= 100, "invalid p: " + std::to_string(p));
            Printer() << "[Scheduler] rebuild_dp: " << p << "%" << (p != 100 ? " bad, " : ", ") << timer << '\n';);
}

// 模拟退火（Simulated Annealing）风格的比较函数。
// 用于判断是否接受一个新的候选解 cur_score。
// 模拟退火思想：允许局部“坏解”，避免陷入局部最优。当 temp 高时，更容易接受坏解；temp 低时，逐渐趋向贪心。
bool SchedulerSolver::compare(double cur_score, double old_score, Randomizer &rnd) const {
    return cur_score <= old_score || rnd.get_d() < std::exp(((old_score - cur_score) / old_score) / temp);
}

// 距离 = agent到task起点距离 *5 + 任务代价
uint64_t SchedulerSolver::get_dist(uint32_t r, uint32_t t) const {
    if (t == -1) {
        return 1e6;
    }

    uint32_t source = get_robots_handler().get_robot(r).node;
    uint64_t dist_to_target = get_hm().get(source, task_target[t]);
    uint64_t dist = dist_to_target * 5 + task_metric[t]; // agent到task起点距离占据最大权重
    ASSERT(static_cast<uint32_t>(dist) == dist, "overflow");

    return dist;
}

// 从机器人 r 移除其任务
void SchedulerSolver::remove(uint32_t r) {
    ASSERT(0 <= r && r < desires.size(), "invalid r");
    uint32_t t = desires[r];
    if (t == -1) {
        return;
    }

    cur_score -= get_dist(r, t); // 总cost
    task_to_robot[t] = -1;
    desires[r] = -1;
}

// 给机器人 r 分配任务 t
void SchedulerSolver::add(uint32_t r, uint32_t t) {
    ASSERT(0 <= r && r < desires.size(), "invalid r");
    ASSERT(0 <= t && t < task_to_robot.size(), "invalid t");
    ASSERT(desires[r] == -1, "already have task");
    ASSERT(task_to_robot[t] == -1, "already have robot");

    cur_score += get_dist(r, t); // 总cost
    task_to_robot[t] = r;
    desires[r] = t;
}

// 随机选择一个机器人 r 和一个任务 t，然后尝试让机器人 r 改执行任务 t（可能涉及任务交换），并通过一次“模拟退火式判断”决定是否接受这个变化。
// ✔ 随机邻域搜索
// ✔ 尝试交换任务
// ✔ 代价下降必接受，代价上升概率接受（SA）
bool SchedulerSolver::try_peek_task(Randomizer &rnd) {
    double old_score = cur_score;

    uint32_t r = rnd.get(free_robots); // 随机选机器人
    uint32_t t = rnd.get(free_tasks); // 随机选任务
    // comment: 这里和吾人的区别是，吾人不是随机挑选，而是选gap最大的那个

    // уже используется
    // 排除“机器人已经在执行该任务”的情况
    if (desires[r] == t) {
        return false;
    }

    // 2️⃣ 准备交换相关的信息
    uint32_t old_t = desires[r];
    uint32_t other_r = task_to_robot[t];
    ASSERT(r != other_r, "invalid other_r");

    // 3️⃣ 尝试执行任务替换（临时修改状态）
    // ▶ 情况 A：任务 t 被 someone 占用（交换任务）
    if (other_r != -1) {
        remove(r);
        remove(other_r);

        add(r, t);
        if (old_t != -1) {
            add(other_r, old_t);
        }
    }
    // ▶ 情况 B：任务 t 没有人占用（单独换任务）
    else {
        remove(r);
        add(r, t);
    }
    validate(); // 笑死了，空函数

    // 4️⃣ 使用模拟退火判断是否接受这次变更
    return consider(old_score, rnd, [&]() {
        if (other_r != -1) {
            remove(r);
            remove(other_r);

            if (old_t != -1) {
                add(r, old_t);
            }
            add(other_r, t);
        } else {
            remove(r);
            if (old_t != -1) {
                add(r, old_t);
            }
        }
        validate();
    });
}

// 笑死了，空函数
void SchedulerSolver::validate() {
    /*std::set<uint32_t> S;
    for (uint32_t r = 0; r < desires.size(); r++) {
        if (desires[r] != -1) {
            ASSERT(!S.count(desires[r]), "already contains");
            S.insert(desires[r]);
            ASSERT(task_to_robot[desires[r]] == r, "invalid task to robot");
        }
    }*/
}

SchedulerSolver::SchedulerSolver(SharedEnvironment *env)
    : env(env), task_to_robot(1'000'000, -1), dp(10'000) {
}

// 负责从环境 env 中读取当前任务和机器人的状态 → 构建可调度的自由任务/机器人集 → 初始化代价 → 为后续搜索做好准备。
void SchedulerSolver::update() {
    // 🧩 1. 初始化 desires / dp / 时间戳等结构
    desires.resize(env->num_of_agents, -1);
    timestep_updated.resize(desires.size());
    dp.resize(desires.size());

    free_robots.clear();
    free_tasks.clear();

    // build free_tasks
    // 🧩 2. 构造 free_tasks —— 可被重新分配的任务
    for (auto &[t, task]: env->task_pool) {
        int r = task.agent_assigned;
        if (
                r == -1// нет агента // 未分配
#ifdef ENABLE_SCHEDULER_CHANGE_TASK
                || task.idx_next_loc == 0// мы можем поменять задачу // 可以重新分配（任务刚开始）
#endif
        ) {
#ifdef ENABLE_SCHEDULER_CHANGE_TASK
            task.agent_assigned = -1;// IMPORTANT! remove task agent assigned
#endif
            free_tasks.push_back(t);
        }
    }

    // build free_robots
    // 🧩 3. 构造 free_robots —— 可以重新分配任务的机器人
    for (uint32_t r = 0; r < env->num_of_agents; r++) {
        int t = env->curr_task_schedule[r];

        // есть задача и она в процессе выполнения
        // не можем ее убрать
        // 情况 A：机器人正在进行任务（不能改变）
        if (env->task_pool.count(t) && env->task_pool.at(t).idx_next_loc != 0) {
            desires[r] = t;
            continue;
        }

        // 情况 B：机器人无任务 或 任务可重分配
        if (
                // нет задачи
                !env->task_pool.count(t)
#ifdef ENABLE_SCHEDULER_CHANGE_TASK
                || env->task_pool.at(t).idx_next_loc == 0
#endif
        ) {
            free_robots.push_back(r);
        }
    }

    // build task_metric, task_target
    {
        ETimer timer;
        for (uint32_t t: free_tasks) {
            if (task_metric.size() <= t) {
                task_metric.resize(t + 1, -1);
                task_target.resize(t + 1);
            }
            auto &task = env->task_pool[t];
            task_target[t] = task.locations[0] + 1;

            uint32_t d = 0;
            for (int i = 0; i + 1 < task.locations.size(); i++) {
                int source = task.locations[i] + 1;
                int target = task.locations[i + 1] + 1;
                d += get_hm().get(get_graph().get_node(Position(source, 0)), target);
            }
            task_metric[t] = d;
        }
    }

    for (uint32_t t: free_tasks) {
        if (t >= task_to_robot.size()) {
            task_to_robot.resize(t + 1, -1);
        }
        task_to_robot[t] = -1;
    }

    cur_score = 0;
    for (uint32_t r: free_robots) {
        desires[r] = -1;
        cur_score += get_dist(r, desires[r]);
    }
    validate();

    PRINT(
            Printer() << "[Scheduler] free robots: " << free_robots.size() << '\n';
            Printer() << "[Scheduler] free tasks: " << free_tasks.size() << '\n';);
}

void SchedulerSolver::lazy_solve(TimePoint end_time) {
    ETimer timer;
    for (uint32_t r: free_robots) {
        remove(r);
    }
    std::unordered_set<uint32_t> used_task;

    auto validate_task = [&](uint32_t task_id) {
        // task is already used
        if (used_task.count(task_id)) {
            return false;
        }
        auto it = env->task_pool.find(task_id);
        if (
                // this task is not available
                it == env->task_pool.end() ||
                // robot already used this task
                it->second.agent_assigned != -1) {
            return false;
        }
        return true;
    };

    if constexpr (!ENABLE_PARALLEL_LAZY_SCHEDULER) {
        // (dist, r, index)
        std::priority_queue<std::tuple<uint32_t, uint32_t, uint32_t>, std::vector<std::tuple<uint32_t, uint32_t, uint32_t>>, std::greater<>> Heap;
        for (uint32_t r: free_robots) {
            if (!dp[r].empty()) {
                Heap.push({dp[r][0].first, r, 0});
            }
        }

        while (!Heap.empty() && get_now() < end_time) {
            auto [dist, r, index] = Heap.top();
            Heap.pop();

            uint32_t task_id = dp[r][index].second;
            ASSERT(dist == dp[r][index].first, "invalid dist");

            if (!validate_task(task_id)) {
                index++;

                if (index < dp[r].size()) {
                    Heap.push({dp[r][index].first, r, index});
                }

                continue;
            }

            ASSERT(env->task_pool.count(task_id), "no contains");
            ASSERT(env->task_pool[task_id].agent_assigned == -1, "already assigned");
            ASSERT(!used_task.count(task_id), "already used");

            add(r, task_id);
            used_task.insert(task_id);
        }
    } else {

        std::vector<std::priority_queue<std::tuple<uint32_t, uint32_t, uint32_t>, std::vector<std::tuple<uint32_t, uint32_t, uint32_t>>, std::greater<>>> Heaps(THREADS);
        for (uint32_t i = 0; i < free_robots.size(); i++) {
            uint32_t r = free_robots[i];
            if (!dp[r].empty()) {
                Heaps[i % THREADS].push({dp[r][0].first, r, 0});
            }
        }

        while (get_now() < end_time) {
            auto work = [&](uint32_t thr) {
                auto &Heap = Heaps[thr];
                while (!Heap.empty() && get_now() < end_time) {
                    auto [dist, r, index] = Heap.top();
                    Heap.pop();

                    uint32_t task_id = dp[r][index].second;
                    ASSERT(dist == dp[r][index].first, "invalid dist");

                    if (!validate_task(task_id)) {
                        index++;

                        if (index < dp[r].size()) {
                            Heap.push({dp[r][index].first, r, index});
                        }

                        continue;
                    }

                    ASSERT(env->task_pool.count(task_id), "no contains");
                    ASSERT(env->task_pool[task_id].agent_assigned == -1, "already assigned");
                    ASSERT(!used_task.count(task_id), "already used");

                    Heap.push({dist, r, index});
                    break;
                }
            };
            std::vector<std::thread> threads(THREADS);
            for (uint32_t thr = 0; thr < threads.size(); thr++) {
                threads[thr] = std::thread(work, thr);
            }
            for (uint32_t thr = 0; thr < threads.size(); thr++) {
                threads[thr].join();
            }
            uint32_t best_i = -1;
            for (uint32_t thr = 0; thr < threads.size(); thr++) {
                if (!Heaps[thr].empty() && (best_i == -1 || std::get<0>(Heaps[best_i].top()) > std::get<0>(Heaps[thr].top()))) {
                    best_i = thr;
                }
            }
            if (best_i == -1) {
                break;
            }

            auto& Heap = Heaps[best_i];
            auto [dist, r, index] = Heap.top();

            uint32_t task_id = dp[r][index].second;
            ASSERT(dist == dp[r][index].first, "invalid dist");

            if (!validate_task(task_id)) {
                continue;
            }

            ASSERT(env->task_pool.count(task_id), "no contains");
            ASSERT(env->task_pool[task_id].agent_assigned == -1, "already assigned");
            ASSERT(!used_task.count(task_id), "already used");

            Heap.pop();
            add(r, task_id);
            used_task.insert(task_id);
        }
    }

    validate();

    {
        uint32_t cnt_assigned = 0;
        for (uint32_t r: free_robots) {
            if (desires[r] != -1) {
                cnt_assigned++;
            }
        }
        uint32_t p = cnt_assigned * 100.0 / free_robots.size();
        ASSERT(0 <= p && p <= 100, "invalid p: " + std::to_string(p));
        Printer() << "[Scheduler] real assigned robots: " << p << "%" << (p != 100 ? " bad\n" : "\n");
    }

    // если робот без задачи, то мы ее дадим. это нужно для WPPL, который не может без цели, он не самурай
    auto it = free_tasks.begin();
    for (uint32_t r = 0; r < desires.size(); r++) {
        if (desires[r] == -1) {
            while (it != free_tasks.end() && task_to_robot[*it] != -1) {
                it++;
            }
            ASSERT(it != free_tasks.end(), "unable to set task");
            add(r, *it);
        }
    }

    PRINT(Printer() << "[Scheduler] lazy solve: " << timer << '\n';);
}

void SchedulerSolver::lns_solve(TimePoint end_time) {
    if (free_robots.empty() || free_tasks.empty() || SCHEDULER_LNS_SOLVE_TIME == 0) {
        return;
    }
    static Randomizer rnd;
    temp = 1;
    ETimer timer;
    double old_score = get_score();
    uint32_t step = 0;
    for (; get_now() < end_time; step++) {
        try_peek_task(rnd);
        temp *= 0.999;
    }
    PRINT(Printer() << "[Scheduler] lns solve: " << old_score << "->" << get_score() << " (" << (old_score - get_score() >= 0 ? "+" : "-") << (old_score - get_score()) / old_score * 100 << "%), " << step << ", " << timer << '\n';);
}

std::vector<int> SchedulerSolver::get_schedule() const {
    std::vector<int> result(desires.size());
    for (uint32_t r = 0; r < desires.size(); r++) {
        result[r] = static_cast<int>(desires[r]);
    }
    return result;
}

double SchedulerSolver::get_score() const {
    return cur_score;
}
