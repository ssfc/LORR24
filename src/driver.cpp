#include "CompetitionSystem.h"
#include "Evaluation.h"
#include "global.h"
#include "nlohmann/json.hpp"
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <climits>
#include <csignal>
#include <memory>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/info.hpp>
#include <Tools/tools.hpp>
#include <settings.hpp>


#ifdef PYTHON
#if PYTHON
#include "pyEntry.hpp"
#include "pyMAPFPlanner.hpp"
#include "pyTaskScheduler.hpp"
#include <pybind11/embed.h>
#endif
#endif

namespace po = boost::program_options;
using json = nlohmann::json;

po::variables_map vm;
std::unique_ptr<BaseSystem> system_ptr;

void sigint_handler(int a) {
    fprintf(stdout, "stop the simulation...\n");
    system_ptr->saveResults(vm["output"].as<std::string>(), vm["outputScreen"].as<int>());
    _exit(0);
}


int main(int argc, char **argv) {
#ifdef PYTHON
#if PYTHON
    pybind11::initialize_interpreter();
#endif
#endif
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()("help", "produce help message")                                                                                                                                                                                        //
            ("unique_id,u", po::value<uint32_t>()->default_value(0), "my unique id for unique launch")                                                                                                                                        //
            ("planner_algo,pa", po::value<string>()->required(), "planner algo")                                                                                                                                                              //
            ("graph_guidance,gg", po::value<string>()->required(), "graph guidance")                                                                                                                                                          //
            ("scheduler_algo,sa", po::value<string>()->required(), "scheduler algo")                                                                                                                                                          //
            ("disable_agents,da", po::value<uint32_t>()->default_value(0), "disable agents num")                                                                                                                                             //
            ("inputFile,i", po::value<std::string>()->required(), "input file name")                                                                                                                                                          //
            ("output,o", po::value<std::string>()->default_value("./output.json"), "output results from the evaluation into a JSON formated file. If no file specified, the default name is 'output.json'")                                   //
            ("outputScreen,c", po::value<int>()->default_value(1), "the level of details in the output file, 1--showing all the output, 2--ignore the events and tasks, 3--ignore the events, tasks, errors, planner times, starts and paths")//
            ("evaluationMode,m", po::value<bool>()->default_value(false), "evaluate an existing output file")                                                                                                                                 //
            ("simulationTime,s", po::value<int>()->default_value(5000), "run simulation")                                                                                                                                                     //
            ("fileStoragePath,f", po::value<std::string>()->default_value(""), "the large file storage path")                                                                                                                                 //
            ("planTimeLimit,t", po::value<int>()->default_value(1000), "the time limit for planner in milliseconds")                                                                                                                          //
            ("preprocessTimeLimit,p", po::value<int>()->default_value(30000), "the time limit for preprocessing in milliseconds")                                                                                                             //
            ("logFile,l", po::value<std::string>()->default_value(""), "redirect stdout messages into the specified log file")                                                                                                                //
            ("logDetailLevel,d", po::value<int>()->default_value(1), "the minimum severity level of log messages to display, 1--showing all the messages, 2--showing warnings and fatal errors, 3--showing fatal errors only")
            ("squareSize", po::value<int>()->default_value(12), "the partition square size");

    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    get_unique_id() = vm["unique_id"].as<uint32_t>();
    // PRINT(Printer() << "unique_id: " << get_unique_id() << '\n';);

    const std::string plan_algo = vm["planner_algo"].as<std::string>();
    {
        auto is_equal = [&](const std::string &value, const std::string key) {
            if (value.size() != key.size()) {
                return false;
            }
            for (uint32_t i = 0; i < key.size(); i++) {
                if (key[i] != value[i] && key[i] != '*') {
                    return false;
                }
            }
            return true;
        };

        if (plan_algo == "pibt") {
            get_planner_type() = PlannerType::PIBT;
        } else if (plan_algo == "pibt_tf") {
            get_planner_type() = PlannerType::PIBT_TF;
        } else if (is_equal(plan_algo, "epibt(*)")) {
            get_planner_type() = PlannerType::EPIBT;
            ASSERT(plan_algo[6] == '3' || plan_algo[6] == '4' || plan_algo[6] == '5', "invalid EPIBT operation depth");
            get_epibt_operation_depth() = plan_algo[6] - '0';
            ASSERT(3 <= get_epibt_operation_depth() && get_epibt_operation_depth() <= 5, "invalid EPIBT operation depth");
        } else if (is_equal(plan_algo, "epibt(*)_lns")) {
            get_planner_type() = PlannerType::EPIBT_LNS;
            ASSERT(plan_algo[6] == '3' || plan_algo[6] == '4' || plan_algo[6] == '5', "invalid EPIBT operation depth");
            get_epibt_operation_depth() = plan_algo[6] - '0';
            ASSERT(3 <= get_epibt_operation_depth() && get_epibt_operation_depth() <= 5, "invalid EPIBT operation depth");
        } else if (is_equal(plan_algo, "pepibt(*)_lns")) {
            get_planner_type() = PlannerType::PEPIBT_LNS;
            ASSERT(plan_algo[7] == '3' || plan_algo[7] == '4' || plan_algo[7] == '5', "invalid EPIBT operation depth");
            get_epibt_operation_depth() = plan_algo[7] - '0';
            ASSERT(3 <= get_epibt_operation_depth() && get_epibt_operation_depth() <= 5, "invalid EPIBT operation depth");
        } else if (plan_algo == "wppl") {
            get_planner_type() = PlannerType::WPPL;
        } else {
            FAILED_ASSERT("unexpected planner algo: " + plan_algo);
        }
    }

    std::string graph_guidance_type = vm["graph_guidance"].as<std::string>();
    {
        if (graph_guidance_type == "enable") {
            graph_guidance_type = "+gg";
            get_graph_guidance_type() = GraphGuidanceType::ENABLE;
        } else if (graph_guidance_type == "disable") {
            graph_guidance_type = "";
            get_graph_guidance_type() = GraphGuidanceType::DISABLE;
        } else {
            FAILED_ASSERT("unexpected graph guidance type");
        }
    }

    std::string scheduler_algo = vm["scheduler_algo"].as<std::string>();
    {
        if (scheduler_algo == "greedy")
        {
            scheduler_algo = "+gs";
            get_scheduler_type() = SchedulerType::GREEDY;
        }
        else if (scheduler_algo == "hungarian")
        {
            scheduler_algo = "+hs";
            get_scheduler_type() = SchedulerType::HUNGARIAN;
        }
        else if (scheduler_algo == "DefaultGreedy")
        {
            scheduler_algo = "+default_greedy";
            get_scheduler_type() = SchedulerType::DEFAULT_GREEDY;
        }
        else if (scheduler_algo == "adaptive_jam_curr_pickup_intersect_curr_goal")
        {
            scheduler_algo = "+adaptive_jam_curr_pickup_intersect_curr_goal";
            get_scheduler_type() = SchedulerType::adaptive_jam_curr_pickup_intersect_curr_goal;
        }
        else if (scheduler_algo == "adaptive_jam_task_pickup_region_count_current")
        {
            scheduler_algo = "+adaptive_jam_task_pickup_region_count_current";
            get_scheduler_type() = SchedulerType::adaptive_jam_task_pickup_region_count_current;
        }
        else
        {
            FAILED_ASSERT("unexpected scheduler algo");
        }
    }

    get_disable_agents() = vm["disable_agents"].as<uint32_t>();

    ASSERT(!(get_planner_type() == PlannerType::WPPL && get_graph_guidance_type() == GraphGuidanceType::DISABLE), "unexpected configuration: only WPPL+GG");
    ASSERT(!(get_planner_type() == PlannerType::PIBT_TF && get_graph_guidance_type() == GraphGuidanceType::ENABLE), "unexpected configuration: only PIBT+traffic flow without GG");

    po::notify(vm);

    boost::filesystem::path p(vm["inputFile"].as<std::string>());
    boost::filesystem::path dir = p.parent_path();
    std::string base_folder = dir.string();
    if (base_folder.size() > 0 && base_folder.back() != '/') {
        base_folder += "/";
    }

    int log_level = vm["logDetailLevel"].as<int>();
    if (log_level <= 1)
        log_level = 2;//info
    else if (log_level == 2)
        log_level = 3;//warning
    else
        log_level = 5;//fatal

    Logger *logger = new Logger(vm["logFile"].as<std::string>(), log_level);

    std::filesystem::path filepath(vm["output"].as<std::string>());
    if (filepath.parent_path().string().size() > 0 && !std::filesystem::is_directory(filepath.parent_path())) {
        logger->log_fatal("output directory does not exist", 0);
        _exit(1);
    }


    Entry *planner = nullptr;

#ifdef PYTHON
#if PYTHON
    planner = new PyEntry();
#else
    planner = new Entry();
#endif
#endif

    auto input_json_file = vm["inputFile"].as<std::string>();
    json data;
    std::ifstream f(input_json_file);
    try {
        data = json::parse(f);
    } catch (json::parse_error error) {
        std::cout << "Failed to load " << input_json_file << std::endl;
        std::cout << "Message: " << error.what() << std::endl;
        exit(1);
    }

    auto map_path = read_param_json<std::string>(data, "mapFile");
    Grid grid(base_folder + map_path);

    planner->env->map_name = map_path.substr(map_path.find_last_of("/") + 1);


    string file_storage_path = vm["fileStoragePath"].as<std::string>();
    if (file_storage_path == "") {
        char const *tmp = getenv("LORR_LARGE_FILE_STORAGE_PATH");
        if (tmp != nullptr) {
            file_storage_path = string(tmp);
        }
    }

    // check if the path exists;
    if (file_storage_path != "" && !std::filesystem::exists(file_storage_path)) {
        std::ostringstream stringStream;
        stringStream << "fileStoragePath (" << file_storage_path << ") is not valid";
        logger->log_warning(stringStream.str());
    }
    planner->env->file_storage_path = file_storage_path;

    ActionModelWithRotate *model = new ActionModelWithRotate(grid);
    model->set_logger(logger);

    int team_size = read_param_json<int>(data, "teamSize");

    std::vector<int> agents = read_int_vec(base_folder + read_param_json<std::string>(data, "agentFile"), team_size);
    std::vector<list<int>> tasks = read_int_vec(base_folder + read_param_json<std::string>(data, "taskFile"));
    if (agents.size() > tasks.size())
        logger->log_warning("Not enough tasks for robots (number of tasks < team size)");

    system_ptr = std::make_unique<BaseSystem>(grid, planner, agents, tasks, model);

    system_ptr->set_logger(logger);
    system_ptr->set_plan_time_limit(vm["planTimeLimit"].as<int>());
    system_ptr->set_preprocess_time_limit(vm["preprocessTimeLimit"].as<int>());

    system_ptr->set_num_tasks_reveal(read_param_json<float>(data, "numTasksReveal", 1));

    signal(SIGINT, sigint_handler);

    system_ptr->simulate(vm["simulationTime"].as<int>());

    system_ptr->saveResults(vm["output"].as<std::string>(), vm["outputScreen"].as<int>());

    task_assignment_method = vm["scheduler_algo"].as<std::string>();
    partition_square_size = vm["squareSize"].as<int>();
    path_planning_method = vm["planner_algo"].as<std::string>();
    system_ptr->saveMyResults(vm["inputFile"].as<std::string>(), std::to_string(vm["simulationTime"].as<int>()),
                              task_assignment_method, partition_square_size, path_planning_method,
                              vm["outputScreen"].as<int>());


    delete model;
    delete logger;

#ifdef BUILD_META_INFO
    build_meta_info("Tmp/" + plan_algo + graph_guidance_type + scheduler_algo + "/test" + std::to_string(get_unique_id()) + ".json",
                    "Tmp/" + plan_algo + graph_guidance_type + scheduler_algo + "/usage" + std::to_string(get_unique_id()) + ".txt");
#endif
    Printer().get().flush();

    _exit(0);
}

// Compile on ubuntu platform:
// cmake. ; make
// Run on ubuntu platform:
// ./bin/lifelong -i ./example_problems_hse/random.domain/random_32_32_20_400.json -o test.json -s 1000 -t 300 -p 1800000 --pa 'epibt(4)' --gg enable --sa greedy

// Main Round Evaluation Instances
// ./bin/lifelong -i ./2024main-instance/MainRoundEvaluationInstances/random.domain/RANDOM-01.json -o test.json -s 600 -t 1000 -p 1800000 --pa 'epibt(4)' --gg enable --sa greedy -c 3
// ./bin/lifelong -i ./2024main-instance/MainRoundEvaluationInstances/random.domain/RANDOM-02.json -o test.json -s 600 -t 1000 -p 1800000 --pa 'epibt(4)' --gg enable --sa greedy -c 3
// ./bin/lifelong -i ./2024main-instance/MainRoundEvaluationInstances/random.domain/RANDOM-03.json -o test.json -s 800 -t 1000 -p 1800000 --pa 'epibt(4)' --gg enable --sa greedy -c 3
// ./bin/lifelong -i ./2024main-instance/MainRoundEvaluationInstances/random.domain/RANDOM-04.json -o test.json -s 1000 -t 1000 -p 1800000 --pa 'epibt(4)' --gg enable --sa greedy -c 3
// ./bin/lifelong -i ./2024main-instance/MainRoundEvaluationInstances/random.domain/RANDOM-05.json -o test.json -s 2000 -t 1000 -p 1800000 --pa 'epibt(4)' --gg enable --sa greedy -c 3
// ./bin/lifelong -i ./2024main-instance/MainRoundEvaluationInstances/city.domain/CITY-01.json -o test.json -s 3000 -t 1000 -p 1800000 --pa 'epibt(4)' --gg enable --sa greedy -c 3
// ./bin/lifelong -i ./2024main-instance/MainRoundEvaluationInstances/city.domain/CITY-02.json -o test.json -s 3000 -t 1000 -p 1800000 --pa 'epibt(4)' --gg enable --sa greedy -c 3
// ./bin/lifelong -i ./2024main-instance/MainRoundEvaluationInstances/game.domain/GAME.json -o test.json -s 5000 -t 1000 -p 1800000 --pa 'epibt(4)' --gg enable --sa greedy -c 3
// ./bin/lifelong -i ./2024main-instance/MainRoundEvaluationInstances/warehouse.domain/SORTATION.json -o test.json -s 5000 -t 1000 -p 1800000 --pa 'epibt(4)' --gg enable --sa greedy -c 3
// ./bin/lifelong -i ./2024main-instance/MainRoundEvaluationInstances/warehouse.domain/WAREHOUSE.json -o test.json -s 5000 -t 1000 -p 1800000 --pa 'epibt(4)' --gg enable --sa greedy -c 3




