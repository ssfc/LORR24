#include <Planner/pepibt_lns.hpp>

#include <Planner/epibt_lns.hpp>
#include <Tools/tools.hpp>

PEPIBT_LNS::PEPIBT_LNS(const std::vector<Robot> &robots, TimePoint end_time) : robots(robots), end_time(end_time), actions(robots.size(), Action::W) {
}

void PEPIBT_LNS::solve(uint64_t seed) {
    EPIBT_LNS main(get_robots_handler().get_robots(), end_time);

    // (score, actions, time, epibt_steps, lns_steps)
    using ItemType = std::tuple<double, std::vector<Action>, uint32_t, uint32_t, uint32_t>;

    constexpr uint32_t THR = THREADS;
    std::vector<std::vector<ItemType>> results_pack(THR);

    launch_threads(THR, [&](uint32_t thr) {
        uint64_t random_seed = seed * (thr + 1) + 426136423;
        ETimer timer;
        EPIBT_LNS solver = main;
        solver.solve(random_seed);
        auto time = timer.get_ms();
        results_pack[thr].emplace_back(solver.get_score(), solver.get_actions(), time, solver.get_epibt_steps(), solver.get_lns_steps());
    });

    std::vector<ItemType> results;
    for (uint32_t thr = 0; thr < THR; thr++) {
        for (auto &item: results_pack[thr]) {
            results.emplace_back(std::move(item));
        }
    }

    double best_score = -1e300;
    std::sort(results.begin(), results.end(), [&](const auto &lhs, const auto &rhs) {
        return std::get<0>(lhs) > std::get<0>(rhs);
    });

    PRINT(Printer() << "[PEPIBT_LNS] results: ";);
    for (const auto &[score, plan, time, epibt_steps, lns_steps]: results) {
        PRINT(Printer() << "(" << score << ", " << time << "ms, " << epibt_steps << "+" << lns_steps << ") ";);
        if (best_score < score) {
            best_score = score;
            actions = plan;
        }
    }
    PRINT(Printer() << '\n';);
}

std::vector<Action> PEPIBT_LNS::get_actions() const {
    return actions;
}
