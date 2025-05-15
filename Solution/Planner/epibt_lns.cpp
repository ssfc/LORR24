#include <Planner/epibt_lns.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/info.hpp>

bool EPIBT_LNS::consider() {
    return old_score - 1e-6 <= cur_score
// old_score > cur_score
#ifdef ENABLE_LNS_ANNEALING
           || rnd.get_d() < std::exp(-((old_score - cur_score) / old_score) / temp)
#endif
            ;
}

EPIBT_LNS::RetType EPIBT_LNS::try_build(uint32_t r, uint32_t &counter) {
    if (counter % 4 == 0 && get_now() >= end_time) {
        return RetType::REJECTED;
    }

    visited[r] = visited_counter;
    uint32_t old_desired = desires[r];

    for (uint32_t desired: robot_desires[r]) {
        desires[r] = desired;
        uint32_t to_r = get_used(r);
        if (to_r == -1) {
            add_path(r);
            if (consider()) {
                return RetType::ACCEPTED;
            } else {
                remove_path(r);
                desires[r] = old_desired;
                return RetType::REJECTED;
            }
        } else if (to_r != -2) {
            ASSERT(0 <= to_r && to_r < robots.size(), "invalid to_r");

            if (visited[to_r] == visited_counter || rnd.get_d() < 0.3 || counter > 3'000) {
                continue;
            }

            remove_path(to_r);
            add_path(r);

            RetType res = try_build(to_r, ++counter);
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

void EPIBT_LNS::try_build(uint32_t r) {
    ++visited_counter;
    old_score = cur_score;
    remove_path(r);
    uint32_t counter = 0;
    if (try_build(r, counter) != RetType::ACCEPTED) {
        add_path(r);
    }
}

EPIBT_LNS::EPIBT_LNS(const std::vector<Robot> &robots, TimePoint end_time)
    : EPIBT(robots, end_time) {
}

void EPIBT_LNS::solve(uint64_t seed) {
    EPIBT::solve();

    rnd = Randomizer(seed);
    temp = 0.001;
    while (get_now() < end_time) {
        uint32_t r = rnd.get(0, robots.size() - 1);
        try_build(r);
        temp *= 0.999;
        lns_step++;
    }
}

void EPIBT_LNS::parallel_solve(uint64_t seed) {
    rnd = Randomizer(seed);
    temp = 0.001;
    while (get_now() < end_time && lns_step < robots.size() * 10) {
        uint32_t r = rnd.get(0, robots.size() - 1);
        try_build(r);
        temp *= 0.999;
        lns_step++;
    }
}

uint32_t EPIBT_LNS::get_lns_steps() const {
    return lns_step;
}
