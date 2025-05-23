#include <nlohmann/json.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Basic/time.hpp>

#include <fstream>

using json = nlohmann::json;

void call(const std::string &test, int steps_num, const std::string &plan_algo, const std::string &graph_guidance_type, const std::string &scheduler_algo, uint32_t test_id) {
    const std::string algo_name = plan_algo +
                                  (graph_guidance_type == "enable" ? "+gg" : "") +
                                  (scheduler_algo == "greedy" ? "+gs" : (scheduler_algo == "hungarian" ? "+hs" : (FAILED_ASSERT("invalid scheduler_algo"), "")));

    std::cout << "call(" + std::to_string(test_id) + ", " << algo_name << "): " << std::flush;
    ETimer timer;

    std::string call_str = std::string("./bin/lifelong") +                                         //
                           " -i " + test +                                                         //
                           " -o 'Tmp/" + algo_name + "/test" + std::to_string(test_id) + ".json'" +//
                           " -s " + std::to_string(steps_num) +                                    //
                           " -t 2000 " +                                                           //
                           " -p 1000000000" +                                                      //
                           " --unique_id " + std::to_string(test_id) +                             //
                           " --planner_algo '" + plan_algo + "'" +                                 //
                           " --graph_guidance " + graph_guidance_type +                            //
                           " --scheduler_algo " + scheduler_algo +                                 //
                           " > 'Tmp/" + algo_name + "/output" + std::to_string(test_id) + ".txt'";

    //std::cout << "\n>" << call_str << "<" << std::endl;
    int ret_code = std::system(call_str.c_str());

    std::cout << timer << std::endl;
    ASSERT(ret_code == 0, "invalid ret code");
    if (ret_code != 0) {
        //call(test, steps_num, plan_algo, graph_guidance_type, scheduler_algo, test_id);
    }
}

std::vector<std::tuple<std::string, int, bool>> tests = {

        /*{"example_problems/random.domain/random_32_32_20_100.json", 1000, true},
        {"example_problems/random.domain/random_32_32_20_200.json", 1000, true},
        {"example_problems/random.domain/random_32_32_20_300.json", 1000, true},
        {"example_problems/random.domain/random_32_32_20_400.json", 1000, true},
        {"example_problems/random.domain/random_32_32_20_500.json", 1000, true},
        {"example_problems/random.domain/random_32_32_20_600.json", 1000, true},
        {"example_problems/random.domain/random_32_32_20_700.json", 1000, true},
        {"example_problems/random.domain/random_32_32_20_800.json", 1000, true},*/

        /*{"example_problems/warehouse.domain/warehouse_large_1000.json", 5000, true},
        {"example_problems/warehouse.domain/warehouse_large_2000.json", 5000, true},
        {"example_problems/warehouse.domain/warehouse_large_3000.json", 5000, true},
        {"example_problems/warehouse.domain/warehouse_large_4000.json", 5000, true},
        {"example_problems/warehouse.domain/warehouse_large_5000.json", 5000, true},
        {"example_problems/warehouse.domain/warehouse_large_6000.json", 5000, true},
        {"example_problems/warehouse.domain/warehouse_large_7000.json", 5000, true},
        {"example_problems/warehouse.domain/warehouse_large_8000.json", 5000, true},
        {"example_problems/warehouse.domain/warehouse_large_9000.json", 5000, true},
        {"example_problems/warehouse.domain/warehouse_large_10000.json", 5000, true},*/

        {"example_problems/game.domain/brc202d_1000.json", 5000, true},
        {"example_problems/game.domain/brc202d_2000.json", 5000, true},
        {"example_problems/game.domain/brc202d_3000.json", 5000, true},
        {"example_problems/game.domain/brc202d_4000.json", 5000, true},
        {"example_problems/game.domain/brc202d_5000.json", 5000, true},
        {"example_problems/game.domain/brc202d_6000.json", 5000, true},
        {"example_problems/game.domain/brc202d_7000.json", 5000, true},
        {"example_problems/game.domain/brc202d_8000.json", 5000, true},
        {"example_problems/game.domain/brc202d_9000.json", 5000, true},
        {"example_problems/game.domain/brc202d_10000.json", 5000, true},
};

int main() {

    std::cout << "game" << std::endl;

    std::vector<std::string> planner_algos = {
            //"pibt",
            "epibt(4)",
            "pepibt(4)_lns",
            //"wppl",
            //"pibt_tf",
    };

    std::vector<std::string> graph_guidance_types = {
            "enable",
            "disable",
    };

    std::vector<std::string> scheduler_algos = {
            "greedy",
            //"hungarian",
    };

    if (!std::filesystem::exists("Tmp")) {
        std::filesystem::create_directories("Tmp");
    }

    for (const auto &planner_algo: planner_algos) {
        for (const auto &graph_guidance_type: graph_guidance_types) {
            for (const auto &scheduler_algo: scheduler_algos) {
                std::string dirname = "Tmp/" + planner_algo +
                                      (graph_guidance_type == "enable" ? "+gg" : "") +
                                      (scheduler_algo == "greedy" ? "+gs" : (scheduler_algo == "hungarian" ? "+hs" : (FAILED_ASSERT("invalid scheduler_algo"), "")));
                if (!std::filesystem::exists(dirname)) {
                    std::filesystem::create_directories(dirname);
                }
            }
        }
    }

    for (const auto &planner_algo: planner_algos) {
        for (const auto &graph_guidance_type: graph_guidance_types) {
            for (const auto &scheduler_algo: scheduler_algos) {
                uint32_t counter = 0;
                for (auto [test, steps_num, need_to_call]: tests) {
                    if (need_to_call) {
                        call(test, steps_num, planner_algo, graph_guidance_type, scheduler_algo, counter);
                    }
                    counter++;
                }
            }
        }
    }
}
