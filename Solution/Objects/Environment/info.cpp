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
