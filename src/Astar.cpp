// This file implements A* algorithm class
// Author: Huili Yu
#include "Astar.h"
#include "math.h"
#include <assert.h>
#include <float.h>
#include <queue>
#include <utility>
#include <unordered_map>
#include <iostream>
#include <algorithm>

namespace a_star
{
// Operator overload for equal sign
bool Node::operator==(const Point2D &coordinates){
  return coords.x == coordinates.x && coords.y == coordinates.y;
}

// Set environment
void AStar::SetEnvironment(Point2D &upper_left_corners,
                           Point2D &length_width,
                           Point2D &cell_size)
{
  upper_left_corners_ = upper_left_corners;
  length_width_ = length_width;
  cell_size_ = cell_size;
}

// Perform cell decomposition
void AStar::CellDecomposition()
{
  for (int i = upper_left_corners_.y; 
       i < upper_left_corners_.y + length_width_.y; 
       i += cell_size_.y) {
    std::vector<Point2D> tmp;
    for (int j = upper_left_corners_.x; 
         j < upper_left_corners_.x + length_width_.x;
         j = j + cell_size_.x) {
      tmp.push_back(Point2D(j, i));
    }
    cells_.push_back(tmp);
  }
}

// Generate graph
void AStar::GenerateGraph()
{
  int n_rows = (int)cells_.size();
  int n_cols = (int)cells_.front().size();
  float diag_dist = (float)sqrt(cell_size_.x*cell_size_.x + 
                                cell_size_.y*cell_size_.y);
  graph_.AllocatedMemory(n_rows * n_cols);
  for (int i = 0; i < n_rows; i++) {
    for (int j = 0; j < n_cols; j++) {
      // Eight way connectivity
      // For each cell, only consider right, 
      // down right, up right, and down directions
      if (j + 1 < n_cols) {
        if (occupied_[i][j] || occupied_[i][j + 1]) {
	  graph_.AddEdge(i * n_cols + j, 
                         i * n_cols + j + 1, FLT_MAX);  // right
	}
        else {
          graph_.AddEdge(i * n_cols + j, i * n_cols + j + 1, 
                         1.0f * cell_size_.x);  // right
        }
      }
      if (i + 1 < n_rows && j + 1 < n_cols) {
        if (occupied_[i][j] || occupied_[i + 1][j + 1]) {
	  graph_.AddEdge(i * n_cols + j, (i + 1) * n_cols + j + 1, 
                         FLT_MAX);  // down right
        }
        else {
          graph_.AddEdge(i * n_cols + j, (i + 1) * n_cols + j + 1, 
                         diag_dist);  // down right
        }
      }
      if (i - 1 >= 0 && j + 1 < n_cols) {
        if (occupied_[i][j] || occupied_[i - 1][j + 1]) {
          graph_.AddEdge(i * n_cols + j, (i - 1) * n_cols + j + 1, 
                         FLT_MAX);  // up right
        }
        else {
          graph_.AddEdge(i * n_cols + j, (i - 1) * n_cols + j + 1, 
                         diag_dist);  // up right
        }			
      }
      if (i + 1 < n_rows) {
        if (occupied_[i][j] || occupied_[i + 1][j]) {
          graph_.AddEdge(i * n_cols + j, (i + 1) * n_cols + j,
                         FLT_MAX);  // down
	}
        else {
          graph_.AddEdge(i * n_cols + j, (i + 1) * n_cols + j, 
                         1.0f * cell_size_.y);  // down
        }
      }
    }
  }
}

// Set occupied cells
void AStar::SetObstacles(std::vector<std::vector<bool> > &occupied)
{
  occupied_ = occupied;
}

// Compute Euclidean distance between two points
float AStar::ComputeEuclidean(Point2D &source, Point2D &target)
{
  return (float)sqrt((source.x - target.x)*(source.x - target.x) + 
         (source.y - target.y)*(source.y - target.y));
}

// Find the shortest path between two points
float AStar::FindShortestPath(Point2D &start_point,
                              Point2D &end_point,
                              std::vector<int> &path)
{
  // Check coordinates of start and end points
  assert(start_point.x >= upper_left_corners_.x && 
         start_point.x < upper_left_corners_.x + length_width_.x && 
         start_point.y >= upper_left_corners_.y && 
         start_point.y < upper_left_corners_.y + length_width_.y);
  assert(end_point.x >= upper_left_corners_.x && 
         end_point.x < upper_left_corners_.x + length_width_.x && 
         end_point.y >= upper_left_corners_.y && 
         end_point.y < upper_left_corners_.y + length_width_.y);
  assert(!(start_point.x == end_point.x && 
           start_point.y == end_point.y));

  // Initialization
  std::priority_queue<Node, std::vector<Node>, compare> open_set;
  std::unordered_map<int, bool> closed_set;

  int num_of_nodes = graph_.GetNumberOfGraphNodes();
  std::vector<int> prev(num_of_nodes, -1);
  std::vector<float> g_score(num_of_nodes, FLT_MAX);
  std::vector<float> f_score(num_of_nodes, FLT_MAX);
  int x_idx = (start_point.x - upper_left_corners_.x) / cell_size_.x;
  int y_idx = (start_point.y - upper_left_corners_.y) / cell_size_.y;
  int node_idx = y_idx * (int)cells_.size() + x_idx;
  g_score[node_idx] = 0;
  f_score[node_idx] = ComputeEuclidean(start_point, end_point);
  Node node(node_idx, start_point, f_score[node_idx], g_score[node_idx]);
  open_set.push(node);

  // Main loop
  int n_cols = (int)cells_.front().size();
  while (!open_set.empty()) {
    Node curr = open_set.top();
    open_set.pop();
    // If the set is equal to the goal, break the loop
    if (curr == end_point) {
      break;
    }
    // Insert the node index to closed set
    closed_set.insert(std::make_pair(curr.node_idx, true));

    // Extract neighbors
    std::vector<GraphNode *> adj_list = 
        graph_.ExtractNeighbors(curr.node_idx)->adj_list;
    
    // Go through
    for (int i = 0; i < adj_list.size(); i++) {
      int v = adj_list[i]->label;
      // Skip the neighbors that are already in the closed set
      if (closed_set.find(v) != closed_set.end()) {
        continue;
      }
      float weight = adj_list[i]->weight;
      if (weight < 0) {
        std::cout << "Negative weight is found!" << std::endl;
        exit(EXIT_FAILURE);
      }
      float tentative_score = g_score[curr.node_idx] + weight;

      if (tentative_score < g_score[v]) {
	g_score[v] = tentative_score;
	Point2D tmp_point(v % n_cols, v / n_cols);
	f_score[v] = g_score[v] + ComputeEuclidean(tmp_point, end_point);
	Node tmp_node(v, tmp_point, f_score[v], g_score[v]);
	open_set.push(tmp_node);
	prev[v] = curr.node_idx;
      }
    }
  }

  // Extract the shortest path
  int end_xidx = (end_point.x - upper_left_corners_.x) / cell_size_.x;
  int end_yidx = (end_point.y - upper_left_corners_.y) / cell_size_.y;
  int end_nodeidx = end_yidx * n_cols + end_xidx;
  int tmp_idx = end_nodeidx;
  while (tmp_idx != -1) {
    path.push_back(tmp_idx);
    tmp_idx = prev[tmp_idx];
  }
  std::reverse(path.begin(), path.end());
  return g_score[end_nodeidx];
}

// Print the path
void AStar::PrintPath(std::vector<int> &path)
{
  for (int i = 0; i < path.size(); i++) {
    if (i < path.size() - 1) {
      std::cout << path[i] << "->";
    }
    else {
      std::cout << path[i] << std::endl;
    }
  }
}
}  // namespace a_star
