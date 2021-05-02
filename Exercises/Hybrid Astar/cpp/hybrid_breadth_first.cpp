#include <math.h>
#include <iostream>
#include <algorithm>
#include <vector>
#include "hybrid_breadth_first.h"

// Initializes HBF
HBF::HBF() {}

HBF::~HBF() {}

int HBF::theta_to_stack_number(double theta){
  // Takes an angle (in radians) and returns which "stack" in the 3D 
  //   configuration space this angle corresponds to. Angles near 0 go in the 
  //   lower stacks while angles near 2 * pi go in the higher stacks.
  double new_theta = fmod((theta + 2 * M_PI),(2 * M_PI));
  int stack_number = (int)(round(new_theta * NUM_THETA_CELLS / (2*M_PI))) 
                   % NUM_THETA_CELLS;

  return stack_number;
}

int HBF::idx(double float_num) {
  // Returns the index into the grid for continuous position. So if x is 3.621, 
  //   then this would return 3 to indicate that 3.621 corresponds to array 
  //   index 3.
  return int(floor(float_num));
}

/**
 * returns grid distance to goal 
 */
double HBF::heuristic(const vector<double> &start, vector<int> &goal) {
  return fabs(start[0] - goal[0]) + fabs(start[1] - goal[1]);
}

vector<HBF::maze_s> HBF::expand(HBF::maze_s &state, vector<int> &goal) {
  int g = state.g;
  int f = state.f;
  double x = state.x;
  double y = state.y;
  double theta = state.theta;
    
  // increment cost    
  int g2 = g+1;
  vector<HBF::maze_s> next_states;

  // Loop over delta steering angles
  for(double delta_i = -35; delta_i < 40; delta_i+=5) {
    // Use bicycle model to get next state
    double delta = M_PI / 180.0 * delta_i;
    double omega = SPEED / LENGTH * tan(delta);
    double theta2 = theta + omega;
    if(theta2 < 0) {
      theta2 += 2*M_PI;
    }
    double x2 = x + SPEED * cos(theta);
    double y2 = y + SPEED * sin(theta);
    // Store new state
    HBF::maze_s state2;
    state2.g = g2;
    state2.f = g2 + heuristic({x2, y2}, goal);
    state2.x = x2;
    state2.y = y2;
    state2.theta = theta2;
    next_states.push_back(state2);
  }

  return next_states;
}

vector< HBF::maze_s> HBF::reconstruct_path(
  vector<vector<vector<HBF::maze_s>>> &came_from, vector<double> &start, 
  HBF::maze_s &final) {
  // Work backwards from the final state
  vector<maze_s> path = {final};
  
  int stack = theta_to_stack_number(final.theta);
  // Get the state associated with the final heading
  maze_s current = came_from[stack][idx(final.x)][idx(final.y)];
  
  stack = theta_to_stack_number(current.theta);
  
  double x = current.x;
  double y = current.y;
  // Rewind the path back to the start position
  while(x != start[0] || y != start[1]) {
    // Append the current state 
    path.push_back(current);
    current = came_from[stack][idx(x)][idx(y)];
    x = current.x;
    y = current.y;
    stack = theta_to_stack_number(current.theta);
  }
  
  return path;
}

HBF::maze_path HBF::search(vector< vector<int> > &grid, vector<double> &start, 
                           vector<int> &goal) {
  // Working Implementation of breadth first search. Does NOT use a heuristic
  //   and as a result this is pretty inefficient. Try modifying this algorithm 
  //   into hybrid A* by adding heuristics appropriately.

  /**
   * TODO: Add heuristics and convert this function into hybrid A*
   */
  vector<vector<vector<int>>> closed(
    NUM_THETA_CELLS, vector<vector<int>>(grid[0].size(), vector<int>(grid.size())));
  vector<vector<vector<maze_s>>> came_from(
    NUM_THETA_CELLS, vector<vector<maze_s>>(grid[0].size(), vector<maze_s>(grid.size())));
  double theta = start[2];
  int stack = theta_to_stack_number(theta);
  int g = 0;
  int f = heuristic(start, goal);

  // Initialise search start state
  maze_s state;
  state.g = g;
  state.x = start[0];
  state.y = start[1];
  state.theta = theta;
  state.f = g + heuristic(start, goal);

  // Mark current cell as visited
  closed[stack][idx(state.x)][idx(state.y)] = 1;
  // Store the state (cost, x, y, theta) where the vehicle came from
  came_from[stack][idx(state.x)][idx(state.y)] = state;
  int total_closed = 1;
  // Track open cells
  vector<maze_s> opened = {state};
  bool finished = false;
  // Loops until no more open cells
  while(!opened.empty()) {
    std::sort(opened.begin(), opened.end(), [](maze_s a, maze_s b){ return a.f < b.f; });
    maze_s current = opened[0]; //grab first elment
    opened.erase(opened.begin()); //pop first element

    int x = current.x;
    int y = current.y;

    // Check if goal reached
    if(idx(x) == goal[0] && idx(y) == goal[1]) {
      std::cout << "found path to goal in " << total_closed << " expansions" 
                << std::endl;
      // Store the path taken, cells explored and final state
      maze_path path;
      path.came_from = came_from;
      path.closed = closed;
      path.final = current;

      return path;
    }

    // Expand current state to generate next states
    vector<maze_s> next_state = expand(current, goal);

    //  Validate each next state
    for(int i = 0; i < next_state.size(); ++i) {
      int g2 = next_state[i].g;
      int f2 = next_state[i].f;
      double x2 = next_state[i].x;
      double y2 = next_state[i].y;
      double theta2 = next_state[i].theta;

      // If outside off map
      if((x2 < 0 || x2 >= grid.size()) || (y2 < 0 || y2 >= grid[0].size())) {
        // invalid cell
        continue;
      }

      int stack2 = theta_to_stack_number(theta2);

      // If not already visited and cell not blocked, add to open cells
      if(closed[stack2][idx(x2)][idx(y2)] == 0 && grid[idx(x2)][idx(y2)] == 0) {
        opened.push_back(next_state[i]);
        // Set as visited for the current steering angle
        closed[stack2][idx(x2)][idx(y2)] = 1;
        // Update path taken for current steering angle
        came_from[stack2][idx(x2)][idx(y2)] = current;
        ++total_closed;
      }
    }
  }

  std::cout << "no valid path." << std::endl;
  HBF::maze_path path;
  path.came_from = came_from;
  path.closed = closed;
  path.final = state;

  return path;
}