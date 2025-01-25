#pragma once

#include <Objects/Basic/randomizer.hpp>
#include <Planner/PIBT/pibt2.hpp>

#include <atomic>
#include <map>
#include <unordered_map>
#include <unordered_set>

// Priority Inheritance with BackTracking
// Each robot is assigned an action vector from the pool. Examples: FW, FW, W
// Solver mode
class PIBTS {

    double cur_score = 0;

    const std::vector<Robot> &robots;

    TimePoint end_time;

    // used_edge[edge][depth] = robot id
    std::vector<std::array<uint32_t, DEPTH>> used_edge;

    // used_pos[pos][depth] = robot id
    std::vector<std::array<uint32_t, DEPTH>> used_pos;

    // desires[r] = запланированное действие робота
    std::vector<uint32_t> desires;

    // neighbours[r] = { neighbours }
    std::vector<std::vector<uint32_t>> neighbors;

    // smart_dist_dp[r][desired] = get_smart_dist_IMPL(r, desired)
    std::vector<std::vector<int32_t>> smart_dist_dp;

    // robot_desires[r] = { desired }
    std::vector<std::vector<uint32_t>> robot_desires;

    std::vector<uint32_t> order;

    std::vector<uint32_t> weight;
    uint32_t max_weight = 0;

    std::vector<uint32_t> visited;
    uint32_t visited_counter = 1;

    Randomizer rnd;

    double old_score = 0;

    double temp = 1;

    double visited_bound = 0.8;

    std::vector<uint32_t> best_desires;

    double best_score = -1;

    bool consider();

    [[nodiscard]] bool validate_path(uint32_t r, uint32_t desired) const;

    [[nodiscard]] bool is_free_path(uint32_t r) const;

    [[nodiscard]] EPath get_path(uint32_t r, uint32_t desired) const;

    [[nodiscard]] uint32_t get_used(uint32_t r) const;

    [[nodiscard]] int32_t get_smart_dist_IMPL(uint32_t r, uint32_t desired) const;

    [[nodiscard]] int32_t get_smart_dist(uint32_t r, uint32_t desired) const;

    void update_score(uint32_t r, uint32_t desired, double &cur_score, int sign) const;

    void add_path(uint32_t r);

    void remove_path(uint32_t r);

    // return 0, if failed
    // return 1, if success+accepted
    // return 2, if success+not accepted
    uint32_t try_build(uint32_t r, uint32_t &counter, uint32_t depth);

    bool try_build(uint32_t r);

    // return 0, if failed
    // return 1, if success+accepted
    // return 2, if success+not accepted
    uint32_t try_rebuild_neighbors(uint32_t id, const std::vector<uint32_t>& rids, uint32_t &counter, uint32_t depth);

    bool try_rebuild_neighbors(uint32_t r);

    // return 0, if failed
    // return 1, if success+accepted
    // return 2, if success+not accepted
    uint32_t build(uint32_t r, uint32_t depth, uint32_t &counter);

    bool build(uint32_t r);

    [[nodiscard]] double get_score() const;

public:
    explicit PIBTS(const std::vector<Robot> &robots, TimePoint end_time, uint64_t seed);

    void simulate_pibt();

    [[nodiscard]] std::vector<Action> get_actions() const;

    [[nodiscard]] std::vector<uint32_t> get_desires() const;

    [[nodiscard]] std::vector<int64_t> get_changes() const;

    [[nodiscard]] double get_best_score() const;
};
