# Search algorithm comparisons
There are 5 searching algorithms implementations at `C++`. This project it created for "Introduction to AI".

### Algorithms
* [A* search] 
* [Greedy search]* 
* [Uniform cost search]
* [Breadth first search]
* [Depth first search]

### Installation

Open your favorite Terminal and run these commands. You need `C++` compiler. 
```sh
$ g++ -o executableName main.cpp
```
Run the command:
```sh
$ ./executableName inputFileName algorithmName
```
###### Algorithm Names: 
* DFS: dfs
* BFS : bfs
* UCS : ucs
* Greedy Search : gs
* A* Search : as


### Comparison
Comparison Of Algorithms
Worst solution: DFS. It is expected because DFS algorithm searches because it calls itself recursively from one move until it finds it. There is no optimality in it.
BFS gives the solution with minimum depth because of its nature(mentioned earlier). UCS gives the optimal solution for cost.
Greedy Search is not optimal because it always tries to go to node with minimum heuristic cost but the heuristic cost is not optimal all the time.
A* search gives the optimal solution and combines heuristic cost and cost.

### Heuristic Function
I used Manhattan Distance for calculating heuristic cost since the map is in the grid format. Heuristic cost between 2 points (x,y) and (x’,y’) is `(absolute(x-x’) + absolute(y-y’))`.


   [A* search]: <https://en.wikipedia.org/wiki/A*_search_algorithm>
   [Uniform cost search]: <https://en.wikipedia.org/?title=Uniform-cost_search&redirect=no>
   [Depth first search]: <https://en.wikipedia.org/wiki/Depth-first_search>
   [Breadth first search]: <https://en.wikipedia.org/wiki/Breadth-first_search>
   [Greedy search]: <https://en.wikipedia.org/wiki/Greedy_algorithm>
