#include <TaskScheduler.h>

#include <const.h>
#include <scheduler.h>

#include <Objects/Basic/time.hpp>
#include <Objects/Environment/environment.hpp>

#include <settings.hpp>

/**
 * Initializes the task scheduler with a given time limit for preprocessing.
 * 
 * This function prepares the task scheduler by allocating up to half of the given preprocessing time limit 
 * and adjust for a specified tolerance to account for potential timing errors. 
 * It ensures that initialization does not exceed the allocated time.
 * 
 * @param preprocess_time_limit The total time limit allocated for preprocessing (in milliseconds).
 *
 */
void TaskScheduler::initialize(int preprocess_time_limit) {
#ifdef ENABLE_DEFAULT_SCHEDULER
    //give at most half of the entry time_limit to scheduler;
    //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
    int limit = preprocess_time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::schedule_initialize(limit, env);
#else
    // Default initialization
    int limit = preprocess_time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::schedule_initialize(limit, env);

    agent_task.resize(env->num_of_agents); // initialize all agents to free state

    // HSE initialization
    init_environment(*env);
#endif
}

/**
 * Plans a task schedule within a specified time limit.
 * 
 * This function schedules tasks by calling shedule_plan function in default planner with half of the given time limit,
 * adjusted for timing error tolerance. The planned schedule is output to the provided schedule vector.
 * 
 * @param time_limit The total time limit allocated for scheduling (in milliseconds).
 * @param proposed_schedule A reference to a vector that will be populated with the proposed schedule (next task id for each agent).
 */

void TaskScheduler::plan(int time_limit, std::vector<int> &proposed_schedule) {
    ETimer timer;
#ifdef ENABLE_DEFAULT_SCHEDULER
    //give at most half of the entry time_limit to scheduler;
    //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
    int limit = time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::schedule_plan(limit, proposed_schedule, env);

    PRINT(
            uint32_t cnt = 0;
            for (uint32_t r = 0; r < proposed_schedule.size(); r++) {
                cnt += proposed_schedule[r] != -1;
            };
            uint32_t p = cnt * 100.0 / proposed_schedule.size();
            ASSERT(0 <= p && p <= 100, "invalid p: " + std::to_string(p));
            Printer() << "[Dummy Scheduler] assigned robots: " << p << "%" << (p != 100 ? "bad\n" : "\n");
            Printer() << "[Dummy Scheduler] time: " << timer << '\n';);
#else
    TimePoint end_time = std::min(env->plan_start_time + Milliseconds(time_limit - 10), get_now() + Milliseconds(SCHEDULER_REBUILD_DP_TIME + SCHEDULER_LAZY_SOLVE_TIME + SCHEDULER_LNS_SOLVE_TIME));
    update_environment(*env);

    if (get_scheduler_type() == SchedulerType::DEFAULT_GREEDY)
    {
        int limit = time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
        DefaultPlanner::schedule_plan(limit, proposed_schedule, env);
    }
    else if (get_scheduler_type() == SchedulerType::adaptive_jam_curr_pickup_intersect_curr_goal)
    {
        int limit = time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
        adaptive_jam_curr_pickup_intersect_curr_goal(limit, proposed_schedule);
    }
    else if (get_scheduler_type() == SchedulerType::adaptive_jam_task_pickup_region_count_current)
    {
        int limit = time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
        adaptive_jam_task_pickup_region_count_current(limit, proposed_schedule);
    }
    else
    {
        my_scheduler.plan(end_time, proposed_schedule);
    }
    

    PRINT(
            uint32_t cnt = 0;
            for (uint32_t r = 0; r < proposed_schedule.size(); r++) {
                cnt += proposed_schedule[r] != -1;
            };
            uint32_t p = cnt * 100.0 / proposed_schedule.size();
            ASSERT(0 <= p && p <= 100, "invalid p: " + std::to_string(p));
            Printer() << "[Scheduler] assigned robots: " << p << "%" << (p != 100 ? " bad\n" : "\n");
            Printer() << "[Scheduler] time: " << timer << '\n';);
#endif
}

// compute pickup jam by counting whether other agent-task line intersect with this agent-task line
[[nodiscard]] int TaskScheduler::compute_jam_curr_pickup_intersect_curr_goal(int _agent_id,
                                                                             Point _agent_loc, Point _agent_end)
{
    int sum_jam_weight = 0;

    // cout << "pickup loc " << pickup_twodim.x << " " << pickup_twodim.y << endl;

    for(int j=0;j<env->goal_locations.size();j++)
    {
        if (j != _agent_id && !env->goal_locations[j].empty()) // only consider the agent with goals
        {
            int other_agent_loc = env->curr_states.at(j).location;
            Point other_agent_start{other_agent_loc % env->cols, other_agent_loc / env->cols};

            int other_agent_goal = env->goal_locations[j][0].first;
            Point other_agent_end{other_agent_goal % env->cols, other_agent_goal / env->cols};
            // cout << "other agent goal " << other_agent_goal << " " << other_agent_goal_x << " "
            // << other_agent_goal_y << endl;

            // 统计other agent和它目标点的连线与agent-pickup连线发生交叉的数量
            if(isIntersecting(_agent_loc, _agent_end, other_agent_start, other_agent_end))
            {
                // cout << "task direction: " << agent_task_direction_x << " " << agent_task_direction_y << " "
                // << task_direction_length << endl;

                sum_jam_weight++;
            }
        }
    }

    return sum_jam_weight;
}

// 11: compute pickup jam by counting whether other agent-task line intersect with this agent-task line
void TaskScheduler::adaptive_jam_curr_pickup_intersect_curr_goal(int time_limit, std::vector<int> & proposed_schedule)
{
    // test whether two vectors intersect or not
    /*
    Point startA{0, 0};
    Point endA{0, 2};
    Point startB{2, 0};
    Point endB{2, 2};
    cout << "is intersect: " << isIntersecting(startA, endA, startB, endB) << endl;
     （*/

    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // cout << "task pool size " << env->task_pool.size() << endl;

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        int min_task_dist = INT_MAX; // 完成该任务的理论时间下界
        double min_task_heuristic = DBL_MAX;
        double corresponding_traffic_jam = DBL_MAX;
        count = 0;

        int agent_loc = env->curr_states.at(i).location;
        Point agent_point{agent_loc % env->cols, agent_loc / env->cols};
        // cout << "agent loc " << agent_twodim.x << " " << agent_twodim.y << endl;

        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id : free_tasks)
        {
            // check for timeout every 10 task evaluations
            if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            dist = 0;
            c_loc = env->curr_states.at(i).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            int pickup_loc = env->task_pool[t_id].locations[0];
            Point pickup_point{pickup_loc % env->cols, pickup_loc / env->cols};
            int sum_jam_weight = compute_jam_curr_pickup_intersect_curr_goal(i,
                                                                             agent_point,
                                                                             pickup_point);

            // sum_jam_weight * jam_coefficient = guess delay time
            if (dist + sum_jam_weight * jam_coefficient < min_task_heuristic){
                min_task_i = t_id;
                min_task_dist = dist;
                min_task_heuristic = dist + sum_jam_weight * jam_coefficient;
                corresponding_traffic_jam = sum_jam_weight;
            }
            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            agent_task[i].task_id = min_task_i;
            agent_task[i].min_task_dist  = min_task_dist;
            agent_task[i].task_heuristic = min_task_heuristic;
            agent_task[i].assign_moment = env->curr_timestep; // assign task moment
            agent_task[i].jam_when_assign = corresponding_traffic_jam;
        }
        // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}


// 默认分配算法，用于其他分配算法处理首批任务
void TaskScheduler::greedy_sum_without_newtask(int time_limit, std::vector<int> & proposed_schedule)
{
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // 由于new task已经在调用前插入过了, 所以这里不再插入新任务
    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    int min_task_i, min_task_makespan, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        min_task_makespan = INT_MAX;
        count = 0;

        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id : free_tasks)
        {
            //check for timeout every 10 task evaluations
            if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            dist = 0;
            c_loc = env->curr_states.at(i).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            // update the new minimum makespan
            if (dist < min_task_makespan){
                min_task_i = t_id;
                min_task_makespan = dist;
            }
            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
        }
        // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}


// 1.2: 用地图以12x12方形分割区域, task所在区域中agent的数量作为jam
void TaskScheduler::adaptive_jam_task_pickup_region_count_current(int time_limit, std::vector<int> & proposed_schedule)
{
    cout << "adaptive jam task pickup region count current" << endl;
    // use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // so that the remaining time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // cout << "task pool size " << env->task_pool.size() << endl;

    //*
    for(auto const& element : env->new_freeagents)
    {
        agent_task[element].complete_moment = env->curr_timestep;

        if (agent_task[element].task_id != -1)
        {
            numTaskFinished++;
            /*
            FinishedTask temp;
            temp.task_id = agent_task[element].task_id;
            temp.min_task_dist = agent_task[element].min_task_dist;
            temp.jam_when_assign = agent_task[element].jam_when_assign;
            temp.heuristic_duration = agent_task[element].task_heuristic;
            temp.real_duration = agent_task[element].complete_moment - agent_task[element].assign_moment;
            finished_tasks.emplace_back(temp);
             */

            total_min_span += agent_task[element].min_task_dist;
            total_real_duration += agent_task[element].complete_moment - agent_task[element].assign_moment;
            total_jam += agent_task[element].jam_when_assign;

            cout << "complete task " << agent_task[element].task_id
                 << " minDist " << agent_task[element].min_task_dist
                 << " heuristic " << agent_task[element].task_heuristic
                 << " real " << agent_task[element].complete_moment - agent_task[element].assign_moment
                 << " jam " << agent_task[element].jam_when_assign << endl;
        }

        agent_task[element].task_id = -1;
    }
    //*/

    if(numTaskFinished > 0 && total_jam > 0)
    {
        jam_coefficient = (total_real_duration - total_min_span) / total_jam;
        cout << "current jam coefficient: " << jam_coefficient << endl;
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    cout << "free agent num: " << free_agents.size() << endl;
    // cout << "free task num: " << free_tasks.size() << endl;

    if(free_agents.size() > 50)
    {
        if(first_epoch_done_time == -1)
        {
            // 由于大算例初始任务过多, 在初始任务分配完成前采用默认算法
            greedy_sum_without_newtask(time_limit, proposed_schedule);
            return;
        }
    }
    else
    {
        if(first_epoch_done_time == -1)
        {
            first_epoch_done_time = env->curr_timestep; // 初始阶段的任务分配完毕的时间
        }
    }

    int region_column = 12; // 每个region所占的列数
    int region_row = 12; // 每个region所占的行数

    // 地图有几列region
    int num_region_column = std::ceil((double)env->cols / region_column);
    // 地图有几行region
    int num_region_row = std::ceil((double)env->rows / region_row);

    // 将map分为若干区域, 统计每个区域agent的数量
    vector<int> region_agent_num(num_region_column * num_region_row, 0);

    for (int i=0;i<env->num_of_agents;i++)
    {
        int agent_loc = env->curr_states.at(i).location;
        int agent_loc_x = agent_loc % env->cols;
        int agent_loc_y = agent_loc / env->cols;

        int agent_region_x = agent_loc_x / region_column;
        int agent_region_y = agent_loc_y / region_row;

        region_agent_num[agent_region_y * num_region_column + agent_region_x]++;
    }

    /*
    cout << "region agent num: ";
    for(int i : region_agent_num)
    {
        cout << i << " ";
    }
    cout << endl;
     //*/

    if(env->num_of_agents <= 500) // 小算例直接算
    {
        for (int t_id : env->new_tasks)
        {
            int pickup_loc = env->task_pool[t_id].locations[0];
            int pickup_loc_x = pickup_loc % env->cols;
            int pickup_loc_y = pickup_loc / env->cols;

            int pickup_region_x = pickup_loc_x / region_column;
            int pickup_region_y = pickup_loc_y / region_row;

            task_region[t_id] = pickup_region_y * num_region_column + pickup_region_x;
        }
    }
    else // 大算例
    {
        if(env->curr_timestep == first_epoch_done_time) // 在first_epoch_done_time计算完此时的free tasks
        {
            for (int t_id : free_tasks)
            {
                int pickup_loc = env->task_pool[t_id].locations[0];
                int pickup_loc_x = pickup_loc % env->cols;
                int pickup_loc_y = pickup_loc / env->cols;

                int pickup_region_x = pickup_loc_x / region_column;
                int pickup_region_y = pickup_loc_y / region_row;

                task_region[t_id] = pickup_region_y * num_region_column + pickup_region_x;
            }
        }
        else // first_epoch_done_time之后就只计算新任务
        {
            for (int t_id : env->new_tasks)
            {
                int pickup_loc = env->task_pool[t_id].locations[0];
                int pickup_loc_x = pickup_loc % env->cols;
                int pickup_loc_y = pickup_loc / env->cols;

                int pickup_region_x = pickup_loc_x / region_column;
                int pickup_region_y = pickup_loc_y / region_row;

                task_region[t_id] = pickup_region_y * num_region_column + pickup_region_x;
            }
        }
    }


    int min_task_i, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    auto it = free_agents.begin();
    while (it != free_agents.end())
    {
        // keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        int min_task_dist = INT_MAX; // 完成该任务的理论时间下界
        double min_task_heuristic = DBL_MAX;
        double corresponding_traffic_jam = DBL_MAX;
        count = 0;

        // cout << "agent loc " << agent_twodim.x << " " << agent_twodim.y << endl;

        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id : free_tasks)
        {
            // check for timeout every 10 task evaluations
            if (std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            dist = 0;
            c_loc = env->curr_states.at(i).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            int sum_jam_weight = region_agent_num[task_region[t_id]];

            // cout << "sum jam weight " << sum_jam_weight << endl;

            // sum_jam_weight * jam_coefficient = estimated delay time
            if (dist + sum_jam_weight * jam_coefficient < min_task_heuristic){
                min_task_i = t_id;
                min_task_dist = dist;
                min_task_heuristic = dist + sum_jam_weight * jam_coefficient;
                corresponding_traffic_jam = sum_jam_weight;
            }
            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            agent_task[i].task_id = min_task_i;
            agent_task[i].min_task_dist  = min_task_dist;
            agent_task[i].task_heuristic = min_task_heuristic;
            agent_task[i].assign_moment = env->curr_timestep; // assign task moment
            agent_task[i].jam_when_assign = corresponding_traffic_jam;
        }
        // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    cout << "Task Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
#ifndef NDEBUG
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
#endif
}