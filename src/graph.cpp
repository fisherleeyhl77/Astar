// Graph class implementation
// Author: Huili Yu
#include "graph.h"
#include <assert.h>
#include <iostream>

namespace a_star
{
// Constructor
Graph::Graph(int num_of_nodes)
{
  assert(num_of_nodes > 0);
  num_of_nodes_ = num_of_nodes;

  for (int i = 0; i < num_of_nodes_; i++) {
    adj_lists_.push_back(new AdjList());
  }
}

// Allocate memory for the graph
void Graph::AllocatedMemory(int num_of_nodes)
{
  if (num_of_nodes <= num_of_nodes_) {
    return;
  }
  for (int i = num_of_nodes_; i < num_of_nodes; i++) {
    adj_lists_.push_back(new AdjList());
  }
  num_of_nodes_ = num_of_nodes;
}

// Add edge to the graph
void Graph::AddEdge(int src_nodeidx, int dest_nodeidx, float weight)
{
  assert(src_nodeidx >= 0 && src_nodeidx < num_of_nodes_);
  assert(dest_nodeidx >= 0 && dest_nodeidx < num_of_nodes_);
  assert(weight >= 0);
  assert(src_nodeidx != dest_nodeidx);
  // Add an edge from source node to destination node
  GraphNode *new_node = new GraphNode(dest_nodeidx, weight);
  adj_lists_[src_nodeidx]->adj_list.push_back(new_node);

  // For undirected graph, add an edge from destination
  // node to source node
  new_node = new GraphNode(src_nodeidx, weight);
  adj_lists_[dest_nodeidx]->adj_list.push_back(new_node);
}

// Extract neighbors of a node
AdjList *Graph::ExtractNeighbors(int graph_nodeidx)
{
  assert(graph_nodeidx >= 0 && 
         graph_nodeidx < num_of_nodes_);
  return adj_lists_[graph_nodeidx];
}

// Get the number of graph nodes
int Graph::GetNumberOfGraphNodes()
{
  return num_of_nodes_;
}

// Print the graph node
void Graph::PrintGraph()
{
  for (int i = 0; i < num_of_nodes_; i++) {
    std::vector<GraphNode *> tmp = adj_lists_[i]->adj_list;
    size_t m = tmp.size();
    for (int j = 0; j < m; j++) {
      std::cout << "(" << tmp[j]->label << ", " << 
          tmp[j]->weight << ")" << "-->";
    }
    std::cout << std::endl;
  }
}

// Destructor
Graph::~Graph()
{
  for (int i = 0; i < num_of_nodes_; i++) {
    std::vector<GraphNode *> tmp = adj_lists_[i]->adj_list;
    size_t m = tmp.size();
    for (int j = 0; j < m; j++) {
      delete tmp[j];
    }
    delete adj_lists_[i];
  }
}
}  // namespace a_star

