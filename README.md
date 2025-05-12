# League of Robots Runners 2024

## About
This is a project of the No Man's Sky team. The team took first place in all four tracks, becoming the overall winner of [The League of Robots Runners](http://www.leagueofrobotrunners.org/) competition. The entire history of the solution is located here. As well as recent commits aimed at code refactoring

## Project structure

* The `readme` folder contains ***ADDITIONAL INFORMATION*** about project
* The `Papers` folder contains our ***competition certificates***, a detailed report on the algorithm, and a presentation
* The `Solution` folder contains our solution
* The `Solution2` folder contains the solution of last year's winning team with their *WPPL* algorithm
* The `PlanViz` folder contains visualizer
* The `example_problems` folder contains tests from the competition and my additional tests

## Solution

* The `Solution/Python` folder contains pythons scripts
* The `Solution/Objects/Environment` folder contains Map, Graph, GraphGuidance, GuidanceMap, HeuristicMatrix, Operation, OperationsMap and RobotsHandler structures. All these are environmental objects and are needed for convenient use by the solution. 
* The `Solution/Planner` folder contains the planning algorithms: 
  - *PIBT* - my implementation of the basic algorithm
  - *EPIBT* - the upgraded *PIBT*, which instead of choosing the direction of movement, immediately chooses a path with some depth: 3, 4, 5
  - *EPIBT+LNS* - an add-on over *EPIBT* that adds an annealing simulation algorithm to iteratively improve the solution
  - *PEPIBT+LNS* - multithreaded launch of *EPIBT+LNS*

## How to run

You can compile a project using *CMakeLists.txt* it will create a `bin` folder with executable files. We are interested in *bin/lifelong*

For main information, see the `readme` folder. We also add additional arguments to the *lifelong* program launch:

| options                        |                                                                                                                                                                                                                                                                              |
|--------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| --planner_algo <br /> --pa     | String <br /> "pibt" = PIBT <br /> "pibt_tf" = PIBT+traffic flow <br /> "wppl" = WPPL <br /> "epibt(x)" = EPIBT(x) <br /> "epibt(x)_lns" = EPIBT(x)+LNS <br /> "pepibt(x)_lns" = Parallel EPIBT(x)+LNS <br /> Where x is operation depth: 3, 4, 5. Highly recommend using 4. |
| --graph_guidance <br /> --gg   | String <br /> "enable" = enable Graph Guidance <br /> "disable" = disable Graph Guidance                                                                                                                                                                                     |
| --scheduler_algo <br /> --sa   | String <br /> "greedy" = the multithreaded greedy task scheduler <br /> "hungarian" = the multithreaded Hungarian algorithm <br /> ***(hungarian is not recommended to use)***                                                                                               |

This way you can run combinations of algorithms. But not all of them: ~~PIBT+traffic flow+GG~~ and ~~WPPL~~ are not supported


An example of running a solution on a map `random-32-32-20` with `400` agents, log entry to the `test.json` file, `1000` steps, `300ms` time limit for one step, `30m` preprocessing time limit, planner: `EPIBT` with operation depth = 4, `enable` Graph Guidance, scheduler: `greedy`
```
./bin/lifelong -i ./example_problems/random.domain/random_32_32_20_400.json -o test.json -s 1000 -t 300 -p 1800000 --pa 'epibt(4)' --gg enable --sa greedy
```

The program also includes self-written assertions. An example of the fall of such an assertion:
```
assert failed at /home/straple/LORR24/src/driver.cpp:119
message: "I failed!"

Process finished with exit code 100
```

Also see `Solution/settings.hpp` for global project settings: enabling asserts, outputting solution logs to `std::cout`, changing the number of threads, changing the time allocated to the task scheduler and others

## Experiments

***OUTDATED EXPERIMENTS***. EPIBT(X) shows better results.

Experiments with all logs, plots, and heat maps are [here](https://github.com/Straple/LORR24_experiments)

## TODO
1) Improve the pool of operations. It's not quite complete right now
2) All attempts to make good multithreading failed. Now it's just using parallel launch
3) Operations of length 5 work strangely on the game test. There are some artifacts in the plot. Therefore, the length of 4 is used. Find out why
