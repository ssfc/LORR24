#include <Objects/Environment/info.hpp>

uint32_t &get_unique_id() {
    static uint32_t unique_id = 0;
    return unique_id;
}

MapType &get_map_type() {
    static MapType type = MapType::NONE;
    return type;
}

PlannerType &get_planner_type() {
    static PlannerType type = PlannerType::EPIBT;
    return type;
}

GraphGuidanceType &get_graph_guidance_type() {
    static GraphGuidanceType type = GraphGuidanceType::ENABLE;
    return type;
}

SchedulerType &get_scheduler_type() {
    static SchedulerType type = SchedulerType::GREEDY;
    return type;
}

uint32_t &get_epibt_operation_depth() {
    static uint32_t depth = 0;
    return depth;
}

uint32_t &get_disable_agents() {
    static uint32_t disable_agents = 0;
    return disable_agents;
}
