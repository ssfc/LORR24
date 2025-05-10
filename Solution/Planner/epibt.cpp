#include <Planner/epibt.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/heuristic_matrix.hpp>
#include <Objects/Environment/operations_map.hpp>
#include <Tools/tools.hpp>

bool EPIBT::validate_path(uint32_t r, uint32_t desired) const {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desired && desired < get_operations().size(), "invalid desired");
    uint32_t node = robots[r].node;
    return get_omap().get_poses_path(node, desired)[0] > 0;
}

uint32_t EPIBT::get_used(uint32_t r) const {
    uint32_t answer = -1;

    auto &poses_path = get_omap().get_poses_path(robots[r].node, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        uint32_t to_pos = poses_path[depth];
        if (used_pos[to_pos][depth] != -1) {
            if (answer == -1) {
                answer = used_pos[to_pos][depth];
            } else if (answer != used_pos[to_pos][depth]) {
                return -2;
            }
        }
    }

    const auto &edges_path = get_omap().get_edges_path(robots[r].node, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        uint32_t to_edge = edges_path[depth];
        if (used_edge[to_edge][depth] != -1) {
            if (answer == -1) {
                answer = used_edge[to_edge][depth];
            } else if (answer != used_edge[to_edge][depth]) {
                return -2;
            }
        }
    }
    return answer;
}

int64_t EPIBT::get_smart_dist_IMPL(uint32_t r, uint32_t desired) const {
    ASSERT(!robots[r].is_disable(), "disable agent is deprecated");

    const auto &op = get_operations()[desired];
    const auto &path = get_omap().get_nodes_path(robots[r].node, desired);

    const uint32_t target = robots[r].target;

    int64_t dist = get_hm().get(path.back(), target);

    if (op[get_epibt_operation_depth() - 1] == Action::W) {
        uint32_t node = path[get_epibt_operation_depth() - 2];
        {
            uint32_t to = get_graph().get_to_node(node, 1);
            dist = std::min(dist, static_cast<int64_t>(get_hm().get(to, target)));
        }
        {
            uint32_t to = get_graph().get_to_node(node, 2);
            dist = std::min(dist, static_cast<int64_t>(get_hm().get(to, target)));
        }

        if (op[get_epibt_operation_depth() - 2] == Action::W) {
            uint32_t to = node;
            to = get_graph().get_to_node(to, 1);
            to = get_graph().get_to_node(to, 1);
            dist = std::min(dist, static_cast<int64_t>(get_hm().get(to, target)));
        }
    }

    for (uint32_t d = 0; d < DEPTH; d++) {
        if (get_graph().get_pos(path[d]).get_pos() == target) {
            dist = d;
            dist = -dist;
        }
    }

    dist = dist * 50 - desired;
    return dist;
}

int64_t EPIBT::get_smart_dist(uint32_t r, uint32_t desired) const {
    return smart_dist_dp[r][desired];
}

void EPIBT::update_score(uint32_t r, uint32_t desired, int sign) {
    int64_t diff = get_smart_dist(r, 0) - get_smart_dist(r, desired);
    cur_score += sign * diff * robot_power[r];
}

void EPIBT::add_path(uint32_t r) {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desires[r] && desires[r] < get_operations().size(), "invalid desired");

    const auto &poses_path = get_omap().get_poses_path(robots[r].node, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        uint32_t to_pos = poses_path[depth];
        ASSERT(to_pos < used_pos.size(), "invalid to_pos");
        ASSERT(used_pos[to_pos][depth] == -1, "already used");
        used_pos[to_pos][depth] = r;
    }

    const auto &edges_path = get_omap().get_edges_path(robots[r].node, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        uint32_t to_edge = edges_path[depth];
        if (to_edge) {
            ASSERT(to_edge < used_edge.size(), "invalid to_edge");
            ASSERT(used_edge[to_edge][depth] == -1, "already used");
            used_edge[to_edge][depth] = r;
        }
    }

    update_score(r, desires[r], +1);
}

void EPIBT::remove_path(uint32_t r) {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desires[r] && desires[r] < get_operations().size(), "invalid desired");

    const auto &poses_path = get_omap().get_poses_path(robots[r].node, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        uint32_t to_pos = poses_path[depth];
        ASSERT(to_pos < used_pos.size(), "invalid to_pos");
        ASSERT(used_pos[to_pos][depth] == r, "invalid node");
        used_pos[to_pos][depth] = -1;
    }
    const auto &edges_path = get_omap().get_edges_path(robots[r].node, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        uint32_t to_edge = edges_path[depth];
        if (to_edge) {
            ASSERT(to_edge < used_edge.size(), "invalid to_edge");
            ASSERT(used_edge[to_edge][depth] == r, "invalid edge");
            used_edge[to_edge][depth] = -1;
        }
    }
    update_score(r, desires[r], -1);
}

EPIBT::RetType EPIBT::build(uint32_t r, uint32_t &counter) {
    if (counter % 4 == 0 && get_now() >= end_time) {
        return RetType::REJECTED;
    }

    visited[r] = visited_counter;
    uint32_t old_desired = desires[r];

    for (uint32_t desired: robot_desires[r]) {
        if (get_operation_depth(desired) > available_operation_depth) {
            continue;
        }
        desires[r] = desired;
        uint32_t to_r = get_used(r);
        if (to_r == -1) {
            add_path(r);
            return RetType::ACCEPTED;
        } else if (to_r != -2) {
            ASSERT(0 <= to_r && to_r < robots.size(), "invalid to_r: " + std::to_string(to_r));

            if (visited[to_r] == visited_counter || counter > 3'000) {
                continue;
            }

            remove_path(to_r);
            add_path(r);

            RetType res = build(to_r, ++counter);
            if (res == RetType::ACCEPTED) {
                return res;
            } else if (res == RetType::REJECTED) {
                remove_path(r);
                add_path(to_r);
                desires[r] = old_desired;
                return res;
            }

            remove_path(r);
            add_path(to_r);
        }
    }

    desires[r] = old_desired;
    visited[r] = 0;
    return RetType::FAILED;
}

void EPIBT::build(uint32_t r) {
    remove_path(r);
    uint32_t counter = 0;
    if (build(r, counter) != RetType::ACCEPTED) {
        add_path(r);
    }
}

EPIBT::EPIBT(const std::vector<Robot> &robots, TimePoint end_time)
    : robots(robots), end_time(end_time), desires(robots.size()), visited(robots.size()) {

    ETimer timer;

    {
        order.resize(robots.size());
        iota(order.begin(), order.end(), 0);
        std::stable_sort(order.begin(), order.end(), [&](uint32_t lhs, uint32_t rhs) {
            return robots[lhs].priority < robots[rhs].priority;
        });

        std::vector<int32_t> weight(robots.size());

        for (uint32_t i = 0; i < robots.size(); i++) {
            weight[order[i]] = i;
        }
        int32_t max_weight = robots.size() + 1;

        robot_power.resize(robots.size());
        // const double workload = robots.size() * 1.0 / get_map().get_count_free();
        for (uint32_t r = 0; r < robots.size(); r++) {
            double power = (max_weight - weight[r]) * 1.0 / max_weight;
            if (robots[r].is_disable()) {
                FAILED_ASSERT("robot must be not disable");
                power = 0;
            }
            power = power * power;
            robot_power[r] = power;
        }

        std::stable_sort(order.begin(), order.end(), [&](uint32_t lhs, uint32_t rhs) {
            return std::tie(robot_power[lhs], lhs) > std::tie(robot_power[rhs], rhs);
        });
    }

    {
        std::array<uint32_t, DEPTH> value{};
        for (uint32_t depth = 0; depth < DEPTH; depth++) {
            value[depth] = -1;
        }
        used_pos.resize(get_graph().get_zipes_size(), value);
        used_edge.resize(get_graph().get_edges_size(), value);
    }

    // init smart_dist_dp
    {
        smart_dist_dp.resize(robots.size(), std::vector<int64_t>(get_operations().size()));

        launch_threads(THREADS, [&](uint32_t thr) {
            for (uint32_t r = thr; r < robots.size(); r += THREADS) {
                for (uint32_t desired = 0; desired < get_operations().size(); desired++) {
                    if (!validate_path(r, desired)) {
                        continue;
                    }
                    smart_dist_dp[r][desired] = get_smart_dist_IMPL(r, desired);
                }
            }
        });
    }

    // init robot_desires
    {
        robot_desires.resize(robots.size());

        launch_threads(THREADS, [&](uint32_t thr) {
            for (uint32_t r = thr; r < robots.size(); r += THREADS) {
                // (priority, desired)
                std::vector<std::pair<int64_t, uint32_t>> steps;
                for (uint32_t desired = 1; desired < get_operations().size(); desired++) {
                    if (!validate_path(r, desired)) {
                        continue;
                    }
                    int64_t priority = get_smart_dist(r, desired);
                    steps.emplace_back(priority, desired);
                }
                std::reverse(steps.begin(), steps.end());
                std::stable_sort(steps.begin(), steps.end());
                for (auto [priority, desired]: steps) {
                    robot_desires[r].push_back(desired);
                }
            }
        });
    }

    for (uint32_t r = 0; r < robots.size(); r++) {
        add_path(r);
    }

    PRINT(Printer() << "[EPIBT] create: " << timer << '\n';);
}

void EPIBT::solve() {
    for (available_operation_depth = 3; available_operation_depth <= get_epibt_operation_depth() + 2; available_operation_depth++) {
        for (int launch = 0; launch < 2; launch++) {
            visited_counter++;
            for (uint32_t r: order) {
                if (get_now() >= end_time) {
                    break;
                }
                epibt_step++;
                if (visited[r] != visited_counter) {
                    build(r);
                }
            }
        }
    }
}

double EPIBT::get_score() const {
    return cur_score;
}

std::vector<Action> EPIBT::get_actions() const {
    std::vector<Action> answer(robots.size());
    for (uint32_t r = 0; r < robots.size(); r++) {
        const auto &op = get_operations()[desires[r]];
        answer[r] = op[0];

#ifdef ENABLE_SMART_OPERATION_EXECUTION
        // перебирает набор действий и выбирает лучшее по расстоянию до цели
        auto update_answer = [&](const std::vector<Action> &actions) {
            ASSERT(!actions.empty(), "is empty");
            std::vector<uint32_t> dists;
            for (auto action: actions) {
                dists.push_back(get_hm().get(get_graph().get_to_node(robots[r].node, action), robots[r].target));
            }
            uint32_t best_i = 0;
            for (uint32_t i = 0; i < actions.size(); i++) {
                if (dists[i] < dists[best_i]) {
                    best_i = i;
                }
            }
            answer[r] = actions[best_i];
        };

        // не меняя траекторию мы попробуем другие повороты или ожидание
        if (op[0] == Action::CR || op[0] == Action::CCR) {
            if (op[0] == op[1]) {
                if (op[2] == Action::W) {
                    // CCW, RRW
                    update_answer({Action::W, Action::CR, Action::CCR});
                } else {
                    // CC, RR
                    update_answer({Action::CR, Action::CCR});
                }
            } else if (op[1] == Action::W) {
                // CW, RW
                update_answer({Action::W, op[0]});
            }
        } else if (desires[r] == 0) {
            // WWW
            update_answer({Action::W, Action::CR, Action::CCR});
        }
#endif
    }
    return answer;
}

uint32_t EPIBT::get_epibt_steps() const {
    return epibt_step;
}
