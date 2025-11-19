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
    GREEDY, // HSE的greedy算法, dist_to_target * 5 + task_metric[t]
    SA_NOT_5, // 模拟退火，没有*5
    SA_jam_intersect, // 模拟退火，统计交点数
    HUNGARIAN, // HSE的匈牙利算法
    DEFAULT_GREEDY, // 默认的greedys算法
    adaptive_jam_curr_pickup_intersect_curr_goal, // 数交叉点
    adaptive_jam_task_pickup_region_count_current, // 数区域内agent数量
};

SchedulerType &get_scheduler_type();

uint32_t& get_epibt_operation_depth();

uint32_t& get_disable_agents();
