#include <queue>;
#include <unordered_map>
#include <iostream>;
#include <string>;
#include <limits>;
#include <cstdio>
#include <ctime>
#include <vector>;
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#include <stdio.h>

cudaError_t addWithCuda(int *c, const int *a, const int *b, unsigned int size);

// Dummy kernel
__global__ void addKernel(int *c, const int *a, const int *b)
{
    int i = threadIdx.x;
    c[i] = a[i] + b[i];
}

struct SimpleGrid {
  int ** grid;
  int width, height;

  SimpleGrid(int ** grid, int width, int height) {
    this->grid = grid;
    this->width = width;
    this->height = height;
  }

  int GetCost(std::pair<int, int> from, std::pair<int, int> to) {
    return 1;
  }

  int * operator [](int i) const { return this->grid[i]; }
};


struct pair_hash {
  std::size_t operator () (const std::pair<int, int> &p) const {
    size_t h = (p.first * (std::numeric_limits<int>::max() + 1) + p.second);
    return h;
  }
};


std::unordered_map<std::pair<int, int>, std::pair<int, int>, pair_hash>
astar(SimpleGrid grid, std::pair<int, int> start, std::pair<int, int> goal) {

  // Item to be stored in PQ
  typedef std::pair<int, std::pair<int, int>> PQElement;
  // PQ definition
  std::priority_queue<PQElement, std::vector<PQElement>, std::less<PQElement>> frontier;

  // Insert starting position
  frontier.emplace(0, start);

  // Utility memory lists
  std::unordered_map<std::pair<int, int>, int, pair_hash> costSoFar;
  std::unordered_map<std::pair<int, int>, std::pair<int, int>, pair_hash> cameFrom;

  cameFrom[start] = start;
  costSoFar[start] = 0;

  // Begin searching until the goal is reached or every possible value
  // evaluated with a failure.
  while (!frontier.empty()) {

    // Get a node to expand it.
    int x, y;
    std::tie(x, y) = frontier.top().second;
    frontier.pop();

    // If goal terminate.
    if (x == goal.first && y == goal.second) break;

#ifdef _DEBUG
    std::cout << "Visiting " << x << " " << y << std::endl;
#endif // DEBUG

    // Add neighbors to priority queue if they are passable.
    for (int i = -1; i < 2; i++) {
      for (int j = -1; j < 2; j++) {

        // Ignore itself.
        if ((i == 0) && (j == 0)) { continue; }

        // Check if it is passable and a valid point in the grid.
        if (((y + i >= 0) && (y + i < grid.height))
          && ((x + j) >= 0) && (x + j < grid.width)
          && grid[y + i][x + j] != 0) {

          std::pair<int, int> next = std::make_pair(x + j, y + i);
          int newCost = costSoFar[std::make_pair(x, y)] + 1;

          // If it the position is explored or current is cheaper than existing.
          if ((costSoFar.find(next) == costSoFar.end()) || (newCost < costSoFar[next])) {
            int priority = newCost + (std::abs(x - next.first) + std::abs(y - next.second));
            frontier.emplace(priority, next);
            cameFrom[next] = std::make_pair(x, y);
            costSoFar[next] = newCost;
          }
        }
      }
    }
  }

  // Free memory.
  std::priority_queue<PQElement, std::vector<PQElement>, std::less<PQElement>>().swap(frontier);
  std::unordered_map<std::pair<int, int>, int, pair_hash>().swap(costSoFar);

  return cameFrom;
}


void drawGrid(SimpleGrid grid, std::pair<int, int> goal,
  std::unordered_map<std::pair<int, int>, std::pair<int, int>, pair_hash> parents) {
  std::pair<int, int> current = goal;
  do {
    if (grid[current.second][current.first] != 0) {
      grid[current.second][current.first] = 6;
    }
    else {
      grid[current.second][current.first] = 99;
    }

    if (current == parents[current]) break;

    current = parents[current];
  } while (parents.find(goal) != parents.end());

  for (int i = 0; i < grid.height; i++) {
    for (int j = 0; j < grid.width; j++) {
      std::cout << grid[i][j] << " ";
    }
    std::cout << std::endl;
  }
}


int main() {
  int constantGrid[][10] = {
    { 1, 0, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 0, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 0, 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 0, 1, 0, 1, 0, 0, 1, 1, 1 },
    { 1, 0, 1, 0, 1, 0, 0, 1, 1, 1 },
    { 1, 1, 1, 0, 1, 0, 0, 1, 1, 1 },
    { 1, 1, 1, 0, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 0, 1, 1, 0, 0, 1, 1 },
    { 1, 1, 1, 0, 1, 1, 0, 0, 1, 1 },
    { 1, 1, 1, 0, 1, 1, 1, 1, 1, 1 }
  };


  int ** grid = new int*[10];
  for (int i = 0; i < 10; i++) {
    grid[i] = new int[10];
    for (int j = 0; j < 10; j++) {
      grid[i][j] = constantGrid[i][j];
    }
  }

  SimpleGrid simpleGrid = SimpleGrid(grid, 10, 10);

  std::clock_t start;
  double duration;

  start = std::clock();

  std::unordered_map<std::pair<int, int>, std::pair<int, int>, pair_hash> parents;
  parents = astar(simpleGrid, std::make_pair(0, 0), std::make_pair(9, 9));

  duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;

  std::cout << "Duration: " << duration << " seconds\n";

  drawGrid(simpleGrid, std::make_pair(9, 9), parents);

  std::unordered_map<std::pair<int, int>, std::pair<int, int>, pair_hash>().swap(parents);

  return 0;
}