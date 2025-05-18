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
                           " --disable_agents 4000" +                                              //
                           " > 'Tmp/" + algo_name + "/output" + std::to_string(test_id) + ".txt'";

    // 4000: 21690
    // 3500: 18900
    int ret_code = std::system(call_str.c_str());
    std::cout << timer << std::endl;
    ASSERT(ret_code == 0, "invalid ret code");
}

std::vector<std::tuple<std::string, int, bool>> tests = {

        /*{"Tests/My/random.domain/random_32_32_20_100.json", 1000, true},
        {"Tests/My/random.domain/random_32_32_20_200.json", 1000, true},
        {"Tests/My/random.domain/random_32_32_20_300.json", 1000, true},
        {"Tests/My/random.domain/random_32_32_20_400.json", 1000, true},
        {"Tests/My/random.domain/random_32_32_20_500.json", 1000, true},
        {"Tests/My/random.domain/random_32_32_20_600.json", 1000, true},
        {"Tests/My/random.domain/random_32_32_20_700.json", 1000, true},
        {"Tests/My/random.domain/random_32_32_20_800.json", 1000, true},*/

        /*{"Tests/My/warehouse.domain/warehouse_large_1000.json", 5000, true},
        {"Tests/My/warehouse.domain/warehouse_large_2000.json", 5000, true},
        {"Tests/My/warehouse.domain/warehouse_large_3000.json", 5000, true},
        {"Tests/My/warehouse.domain/warehouse_large_4000.json", 5000, true},
        {"Tests/My/warehouse.domain/warehouse_large_5000.json", 5000, true},
        {"Tests/My/warehouse.domain/warehouse_large_6000.json", 5000, true},
        {"Tests/My/warehouse.domain/warehouse_large_7000.json", 5000, true},
        {"Tests/My/warehouse.domain/warehouse_large_8000.json", 5000, true},
        {"Tests/My/warehouse.domain/warehouse_large_9000.json", 5000, true},
        {"Tests/My/warehouse.domain/warehouse_large_10000.json", 5000, true},*/

        /*{"Tests/My/game.domain/brc202d_1000.json", 5000, true},
        {"Tests/My/game.domain/brc202d_2000.json", 5000, true},
        {"Tests/My/game.domain/brc202d_3000.json", 5000, true},
        {"Tests/My/game.domain/brc202d_4000.json", 5000, true},
        {"Tests/My/game.domain/brc202d_5000.json", 5000, true},
        {"Tests/My/game.domain/brc202d_6000.json", 5000, true},
        {"Tests/My/game.domain/brc202d_7000.json", 5000, true},
        {"Tests/My/game.domain/brc202d_8000.json", 5000, true},
        {"Tests/My/game.domain/brc202d_9000.json", 5000, true},
        {"Tests/My/game.domain/brc202d_10000.json", 5000, true},*/

        {"Tests/Competition/city.domain/CITY-01.json", 3000, false},
        {"Tests/Competition/city.domain/CITY-02.json", 3000, false},
        {"Tests/Competition/game.domain/GAME.json", 5000, true},
        {"Tests/Competition/random.domain/RANDOM-01.json", 600, false},
        {"Tests/Competition/random.domain/RANDOM-02.json", 600, false},
        {"Tests/Competition/random.domain/RANDOM-03.json", 800, false},
        {"Tests/Competition/random.domain/RANDOM-04.json", 1000, false},
        {"Tests/Competition/random.domain/RANDOM-05.json", 2000, false},
        {"Tests/Competition/warehouse.domain/SORTATION.json", 5000, false},// epibt(4)+gg плохой результат, так как плохо шедулер работал, нужно дать больше времени
        {"Tests/Competition/warehouse.domain/WAREHOUSE.json", 5000, false},

        // тестировать только pepibt(4)_lns +-gg
        // нужно дотестировать game
        // и протестировать random
        // warehouse не поменялся
};

int main() {

    std::cout << "kek" << std::endl;

    std::vector<std::string> planner_algos = {
            //"pibt",
            //"epibt(4)",
            "pepibt(4)_lns",
            //"wppl",
            //"pibt_tf",
    };

    std::vector<std::string> graph_guidance_types = {
            "enable",
            //"disable",
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
