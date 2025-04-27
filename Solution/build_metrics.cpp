#include <nlohmann/json.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Basic/time.hpp>

#include <fstream>

using json = nlohmann::json;

std::ofstream table_output;

std::ofstream total_table_output;

void call(const std::string &map_name, const std::string &algo_name, uint32_t test_id) {
    json data;
    std::ifstream input("Data_" + map_name + "/" + algo_name + "/test" + std::to_string(test_id) + ".json");
    if (!input) {
        std::cout << "unable ";
        return;
    }
    double throughput = 0;
    uint32_t avg_step_time = 0;
    uint32_t task_finished = 0;
    uint32_t agents_num = 0;
    try {
        data = json::parse(input);

        uint32_t steps_num = data["makespan"];
        task_finished = data["numTaskFinished"];
        throughput = static_cast<double>(data["numTaskFinished"]) / steps_num;
        std::vector<double> times = data["plannerTimes"];
        avg_step_time = static_cast<uint32_t>(std::accumulate(times.begin(), times.end(), 0.0) * 1000 / steps_num);
        agents_num = data["teamSize"];

        table_output << test_id << ";" << algo_name << ";" << agents_num << ";" << steps_num << ";" << task_finished << ";" << throughput << ";" << avg_step_time << std::endl;
        total_table_output << test_id << ";" << algo_name << ";" << agents_num << ";" << steps_num << ";" << task_finished << ";" << throughput << ";" << avg_step_time << std::endl;

        if (data["numEntryTimeouts"] != 0) {
            std::cerr << "\nENTRY TIMEOUT\n";
        }
        if (data["numPlannerErrors"] != 0) {
            std::cerr << "\nPLANNER ERROR\n";
        }
        if (data["numScheduleErrors"] != 0) {
            std::cerr << "\nSCHEDULER ERROR\n";
        }

    } catch (const json::parse_error &error) {
        std::cerr << "Failed at: "
                  << "Data_" + map_name + "/" + algo_name + "/test" + std::to_string(test_id) + ".json" << std::endl;
        std::cerr << "Message: " << error.what() << std::endl;
    }

    // build usage plots
    {
        int ret_code = std::system(("python3 Solution/Python/build_usage_plot.py Data_" + map_name + "/" + algo_name + "/usage" + std::to_string(test_id) + ".txt Data_" + map_name + "/" + algo_name + "/usage_plot" + std::to_string(test_id) + "_one.pdf Data_" + map_name + "/" + algo_name + "/usage_plot" + std::to_string(test_id) + "_all.pdf").c_str());
        ASSERT(ret_code == 0, "invalid ret code");
    }
}

int main() {

    std::vector<std::string> maps_name = {
            "random",
            //"warehouse",
            //"game",
    };

    std::vector<std::string> plan_algos = {
            //"pibt",
            "epibt",
            //"pibt_tf",
            //"epibt_lns",
            //"pepibt_lns",
            //"wppl",
    };

    std::vector<std::string> graph_guidance_types = {
            "enable",
            //"disable",
    };

    std::vector<std::string> scheduler_algos = {
            "greedy",
            "hungarian",
    };

    for (const auto &map_name: maps_name) {
        total_table_output = std::ofstream("Data_" + map_name + "/total_metrics.csv");
        total_table_output << "id;algo name;agents num;steps num;num task finished;throughput;avg step time\n";
        for (const auto &plan_algo: plan_algos) {
            for (const auto &graph_guidance_type: graph_guidance_types) {
                for (const auto &scheduler_algo: scheduler_algos) {
                    std::string algo_name = plan_algo +
                                            (graph_guidance_type == "enable" ? "+gg" : "") +
                                            (scheduler_algo == "greedy" ? "+gs" : (scheduler_algo == "hungarian" ? "+hs" : (FAILED_ASSERT("invalid scheduler_algo"), "")));
                    table_output = std::ofstream("Data_" + map_name + "/" + algo_name + "/metrics.csv");
                    table_output << "id;algo name;agents num;steps num;num task finished;throughput;avg step time\n";
                    for (uint32_t test_id = 0; test_id < 10; test_id++) {
                        ETimer timer;
                        std::cout << "call(" << map_name << ' ' << algo_name << ' ' << test_id << "): " << std::flush;
                        call(map_name, algo_name, test_id);
                        std::cout << timer << std::endl;
                    }
                }
            }
        }
    }
}
