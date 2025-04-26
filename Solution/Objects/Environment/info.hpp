#pragma once

#include <cstdint>

uint32_t &get_unique_id();

enum class MapType {
    RANDOM,
    GAME,
    CITY,
    WAREHOUSE,
    SORTATION,

    NONE,
};

MapType &get_map_type();

enum class PlannerType {
    PIBT,      // pibt
    PIBT_TF,   // pibt + trafficflow
    EPIBT,     // epibt
    EPIBT_LNS, // epibt + lns
    PEPIBT_LNS,// parallel epibt + lns
    WPPL,      // windowed parallel pibt + lns

    //PIBT_LNS,  // pibt + lns
    //P_PIBT_LNS,// parallel pibt + lns

    NONE,
};

PlannerType &get_planner_type();

enum class GraphGuidanceType {
    ENABLE,
    DISABLE
};

GraphGuidanceType &get_graph_guidance_type();
