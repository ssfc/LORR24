#pragma once

#include <array>
#include <cstdint>
#include <iostream>
#include <vector>

#include <SharedEnv.h>

#include <Objects/Environment/map.hpp>

struct GraphGuidance {
    // weight[pos][dir][action]
    std::vector<std::array<std::array<uint16_t, 4>, 4>> graph;

    uint32_t rows = 0;
    uint32_t cols = 0;

    void set(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint32_t dir, uint32_t action, uint16_t value);

    void set_grid();

    void set_warehouse();

    void set_sortation();

    void set_game();

    void set_city();

public:
    GraphGuidance() = default;

    explicit GraphGuidance(SharedEnvironment &env, const Map &map);

    [[nodiscard]] uint32_t get(uint32_t pos, uint32_t dir, uint32_t action) const;

    void set(uint32_t pos, uint32_t dir, uint32_t action, uint16_t weight);

    friend std::istream &operator>>(std::istream &input, GraphGuidance &gg);

    friend std::ostream &operator<<(std::ostream &output, const GraphGuidance &gg);
};

GraphGuidance &get_gg();
