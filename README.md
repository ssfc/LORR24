# League of Robots Runners 2024

## About
This is a project of the No Man's Sky team. The team took first place in all four tracks, becoming the overall winner of [The League of Robots Runners](http://www.leagueofrobotrunners.org/) competition. The entire history of the solution is located here. As well as recent commits aimed at code refactoring

## Project structure

* The **Papers** folder contains our competition certificates, a detailed report on the algorithm, and a presentation
* The **readme** folder contains additional information about project
* The **Data** folder contains our tests and their logs. Inside the folder is a metrics_plot.pdf, which shows a comparison of the algorithms on these tests. The folder is divided into random, game, and warehouse tests. Each test stores information about 6 algorithms. Each algorithm stores the solution log, program output, metrics, and plots on each test
* The **Solution** folder contains our solution
* The **Solution2** folder contains the solution of last year's winning team with their *WPPL* algorithm
* The **PlanViz** folder contains visualizer
* The **example_problems** folder contains tests from the competition and my additional tests

## Solution

* The **Solution/Python** folder contains pythons scripts
* The **Solution/Objects/Environment** folder contains Map, Graph, GraphGuidance, GuidanceMap, HeuristicMatrix, Operation, OperationsMap and RobotsHandler structures. All these are environmental objects and are needed for convenient use by the solution. 
* The **Solution/Planner** folder contains the planning algorithms: 
  - *PIBT* - my implementation of the basic algorithm
  - *EPIBT* - the upgraded *PIBT*, which instead of choosing the direction of movement, immediately chooses a path with some depth: 3, 4, 5
  - *EPIBT+LNS* - an add-on over *EPIBT* that adds an annealing simulation algorithm to iteratively improve the solution
  - *PEPIBT+LNS* - multithreaded launch of *EPIBT+LNS*

## TODO
1) Improve the pool of operations. It's not quite complete right now
2) All attempts to make good multithreading failed. Now it's just using parallel launch
3) Operations of length 5 work strangely on the game test. There are some artifacts in the plot. Therefore, the length of 4 is used. Find out why
