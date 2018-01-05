#include <iostream>
#include "graph.h"
#include "Astar.h"

int main(int argc, char **argv)
{
  a_star::AStar astar;
  a_star::Point2D upper_left_corners(0, 0);
  a_star::Point2D length_width(10, 10);
  a_star::Point2D cell_size(1, 1);
  astar.SetEnvironment(upper_left_corners, length_width, cell_size);
  astar.CellDecomposition();
  
  // Set obstacles
  int n_rows = length_width.y / cell_size.y;
  int n_cols = length_width.x / cell_size.x;
  std::vector<std::vector<bool> > occupied;
  for (int i = 0; i < n_rows; i++) {
    std::vector<bool> tmp;
    for (int j = 0; j < n_cols; j++) {
      if (j == 5 && i <= 5) {
        tmp.push_back(true);
      }
      else {
        tmp.push_back(false);
      }
    }
    occupied.push_back(tmp);
  }
  astar.SetObstacles(occupied);

  astar.GenerateGraph();
  a_star::Point2D start_point(1, 1);
  a_star::Point2D end_point(9, 0);
  std::vector<int> path;
  float total_length = 
      astar.FindShortestPath(start_point, end_point, path);
  std::cout << total_length << std::endl;
  astar.PrintPath(path);

  std::cout << "End of main" << std::endl;
  return 0;
}
