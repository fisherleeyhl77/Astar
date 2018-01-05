// The graph class using adjacency list
// Author: Huili Yu
#ifndef ASTAR_GRAPH_H_
#define ASTAR_GRAPH_H_

// Headers
#include <vector>
#include "graph.h"

namespace a_star
{
// Types
typedef struct GraphNode
{
  int label;
  float weight;
  GraphNode(int l, float w) : label(l), weight(w) {};
} GraphNode;

typedef struct AdjList
{
  std::vector<GraphNode *> adj_list;
} AdjList;

// Class
class Graph
{
public:
  /*
   * Constructor
   */
  Graph() : num_of_nodes_(0) {}
  
  /*
   * Constructor
   * @param num_of_nodes: 
   */
  Graph(int num_of_nodes);
  
  /*
   * AllocatedMemory allocates memory for the graph
   * based on the number of graph nodes
   * @param num_of_nodes: the number of graph nodes
   */
  void AllocatedMemory(int num_of_nodes);

  /*
   * AddEdge adds an edge between two nodes
   * @param src_node: source node index of the edge
   * @param dest_node: destination node index of the edge
   * @param weight: the weight of the edge
   */
  void AddEdge(int src_node, int dest_node, float weight);

  /*
   * ExtractNeighbors extracts the neighbors of a node
   * @param graph_nodeidx: graph node index
   */
  AdjList *ExtractNeighbors(int graph_nodeidx);

  /*
   * GetNumberOfGraphNodes returns the number
   * of graph nodes
   */
  int GetNumberOfGraphNodes();

  /*
   * PrintGraph prints the graph nodes
   */
  void PrintGraph();

  /*
   * Destructor
   */
  virtual ~Graph();
private:
  int num_of_nodes_;                       // Number of graph nodes
  std::vector<AdjList *> adj_lists_;       // Adjacency lists
};  // namespace a_star
}

#endif  //  ASTAR_GRAPH_H_
