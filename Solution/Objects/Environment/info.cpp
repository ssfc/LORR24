#include <Objects/Environment/info.hpp>

// 返回一个静态变量 unique_id 的引用，可以在程序各处读取或修改它的值。
uint32_t &get_unique_id() {
    static uint32_t unique_id = 0;
    return unique_id;
}

// 提供一个在程序运行期间始终唯一的 MapType 类型变量的引用。
MapType &get_map_type() {
    static MapType type = MapType::NONE;
    return type;
}

// 返回一个静态局部变量 type（类型为 PlannerType）的引用。
PlannerType &get_planner_type() {
    static PlannerType type = PlannerType::EPIBT;
    return type;
}

// 提供一个全局可访问、可修改的 GraphGuidanceType 类型变量的引用，并确保只有这一份变量被使用。
GraphGuidanceType &get_graph_guidance_type() {
    static GraphGuidanceType type = GraphGuidanceType::ENABLE;
    return type;
}

// 提供一个全局唯一的、可读可写的 SchedulerType 类型变量，并通过函数接口进行访问和管理。
SchedulerType &get_scheduler_type() {
    static SchedulerType type = SchedulerType::GREEDY;
    return type;
}

// 返回一个代表操作深度的静态变量的引用。
uint32_t &get_epibt_operation_depth() {
    static uint32_t depth = 0;
    return depth;
}

// 全局管理和访问“禁用的代理（agents）数量”，并允许你随时获取或修改这个数值。
uint32_t &get_disable_agents() {
    static uint32_t disable_agents = 0;
    return disable_agents;
}
