#include <nlohmann/json.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Basic/time.hpp>

#include <fstream>

using json = nlohmann::json;

std::ofstream table_output;

std::ofstream total_table_output;

const std::string dir = "Tmp/";

void call(const std::string &algo_name, uint32_t test_id) {
    json data;
    std::ifstream input(dir + algo_name + "/test" + std::to_string(test_id) + ".json");
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
                  << dir + algo_name + "/test" + std::to_string(test_id) + ".json" << std::endl;
        std::cerr << "Message: " << error.what() << std::endl;
    }

    // build usage plots
    {
        //std::string launch_str = "python3 Solution/Python/build_usage_plot.py '" + dir + algo_name + "/usage" + std::to_string(test_id) + ".txt' heatmap" + std::to_string(test_id) + " " + "game";
        //std::cout << "\nlaunch_str: " << launch_str << std::endl;
        //int ret_code = std::system(launch_str.c_str());
        //ASSERT(ret_code == 0, "invalid ret code");
    }
}

int main() {

    total_table_output = std::ofstream(dir + "total_metrics.csv");
    total_table_output << "id;algo name;agents num;steps num;num task finished;throughput;avg step time\n";

    for (auto const &iter: std::filesystem::directory_iterator(dir)) {
        if (!iter.is_directory()) {
            continue;
        }
        std::string algo_name = iter.path();
        algo_name = algo_name.substr(std::find(algo_name.begin(), algo_name.end(), '/') - algo_name.begin() + 1);

        table_output = std::ofstream(dir + algo_name + "/metrics.csv");
        if (!table_output) {
            std::cout << "skipped: " << algo_name << std::endl;
            continue;
        }
        table_output << "id;algo name;agents num;steps num;num task finished;throughput;avg step time\n";
        for (uint32_t test_id = 0; test_id < 10; test_id++) {
            ETimer timer;
            std::cout << "call(" << algo_name << ' ' << test_id << "): " << std::flush;
            call(algo_name, test_id);
            std::cout << timer << std::endl;
        }
    }
}
