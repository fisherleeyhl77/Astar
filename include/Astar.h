// This file defines A* algorithm class
// Author: Huili Yu
#ifndef ASTAR_ASTAR_H_
#define ASTAR_ASTAR_H_

// Headers
#include <vector>
#include <string>
#include "graph.h"

namespace a_star
{
// Types
typedef unsigned int uint;

// Point in two dimensional space
typedef struct Point2D
{
  int x;
  int y;
  Point2D() {};
  Point2D(int x_coord, int y_coord) : x(x_coord), y(y_coord) {};

} Point2D;

// Node
typedef struct Node
{
  int node_idx;
  Point2D coords;
  float f_score;
  float g_score;
  Node() {};
  Node(int idx, Point2D coords_input,
       float f_score_input, float g_score_input) : node_idx(idx),
       coords(coords_input), f_score(f_score_input), 
       g_score(g_score_input) {};
  bool operator==(const Point2D &coordinates);
} Node;

// For sort
struct compare
{
  bool operator()(const Node &l, const Node &r) const
  {
    return l.f_score > r.f_score;
  }
};

// Class for A* algorithm
class AStar
{
 public:
  /*
   * Constructor
   */
  AStar(){};
 
  /*
   * SetEnvironment sets the environment by providing upper left
   * corner, length and width, and cell size in both x and y directions
   * @param upper_left_corners: x, y coordinates of upper left corner
   * @param length_width: length and width of the environment
   * @param cell_size: cell size in x and y directions
   */
  void SetEnvironment(Point2D &upper_left_corners, 
                      Point2D &length_width, Point2D &cell_size);

  /*
   * CellDecomposition decomposes the environment into cells
   */
  void CellDecomposition();

  /*
   * GenerateGraph connects the cells into a graph
   */
  void GenerateGraph();

  /*
   * SetObstacles sets the occupied cells for A* algorithm
   * @param occupied: the input occupied cells
   */
  void SetObstacles(std::vector<std::vector<bool> > &occupied);

  /*
   * FindShortestPath finds the shortest path between two points
   * @param start_point: start point of the path
   * @param end_point: end point of the path
   * @param path: generated path
   * Return: the shortest distance between the two points
   */
  float FindShortestPath(Point2D &start_point, Point2D &end_point, 
                         std::vector<int> &path);

  /*
   * PrintPath prints the path
   */
  void PrintPath(std::vector<int> &path);
 private:
  /*
   * ComputeEuclidean computes the Euclidean distance between
   * two points
   * Return: the distance between the two points
   */
  float ComputeEuclidean(Point2D &source, Point2D &target);

  Point2D upper_left_corners_;
  Point2D length_width_;
  Point2D cell_size_;
  Graph graph_;
  std::vector<std::vector<Point2D> > cells_;
  std::vector<std::vector<bool> > occupied_;
};
}  // namespace a_star

#endif  //  ASTAR_ASTAR_H_
