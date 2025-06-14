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
};

PlannerType &get_planner_type();

enum class GraphGuidanceType {
    ENABLE,
    DISABLE
};

GraphGuidanceType &get_graph_guidance_type();

enum class SchedulerType {
    GREEDY,
    HUNGARIAN,
    DEFAULT_GREEDY,
    adaptive_jam_curr_pickup_intersect_curr_goal,
    adaptive_jam_task_pickup_region_count_current,
};

SchedulerType &get_scheduler_type();

uint32_t& get_epibt_operation_depth();

uint32_t& get_disable_agents();
