#pragma once

#include <string>
#include <thread>
#include <vector>

void build_meta_info(const std::string &from, const std::string &to);

template<typename foo_t>
void launch_threads(const uint32_t threads_num, foo_t &&foo) {
    std::vector<std::thread> threads(threads_num);
    for (uint32_t thr = 0; thr < threads.size(); thr++) {
        threads[thr] = std::thread(foo, thr);
    }
    for (uint32_t thr = 0; thr < threads.size(); thr++) {
        threads[thr].join();
    }
}
