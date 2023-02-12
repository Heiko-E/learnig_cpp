#include <iostream>
#include <vector>
#include <map>
#include <bits/stdc++.h>
using namespace std;
template <class T>
class PriorityQueue
/// @brief The value of the PriorityQueue is to always have access to the vertex
///        with the next shortest link in the shortest path calculation at the top of the queue.

{
public:
  PriorityQueue() {}

  ~PriorityQueue() {}

  bool setMinPrioirity(const T &queueElement, int priority)
  {
    /// @brief changes the priority and path of a queue element to the minimum.
    /// @param queueElement the element to change
    /// @param priority the new priority
    /// @return true if the update succeeded

    typename list<pair<T, int>>::iterator it;
    for (it = this->queue_.begin(); it != this->queue_.end(); it++)
    {
      if (it->first == queueElement)
      {
        if (it->second > priority)
        {
          it->second = priority;
          it->first = queueElement;
          this->queue_.sort([](pair<T, int> a, pair<T, int> b)
                            { return a.second < b.second; });
          return true;
        }
        return false;
      }
    }
    return false;
  }

  int minPrioirty()
  {
    /// @brief priority of the top element of the queue.
    /// @return the pririty of the top element.
    return this->queue_.front().second;
  }

  bool contains(const T &queueElement)
  {
    /// @brief does the queue contains the element.
    /// @param queueElement the element to look for
    /// @return true if element is in the queue

    for (auto element : this->queue_)
    {
      if (element.first == queueElement)
      {
        return true;
      }
    }
    return false;
  }

  bool insert(const T &queueElement, int priority)
  {
    /// @brief insert node into queue
    /// @param queueElement the node to insert
    /// @param priority the priority of the node
    /// @return true if the insertion succeeded

    if (this->contains(queueElement))
    {
      return this->setMinPrioirity(queueElement, priority);
    }
    else
    {
      for (auto it = this->queue_.begin(); it != this->queue_.end(); it++)
      {
        if (it->second > priority)
        {
          this->queue_.insert(it, pair<T, int>(queueElement, priority));
          return true;
        }
      }
    }
    this->queue_.push_back(pair<T, int>(queueElement, priority));
    return true;
  }

  T popTop()
  {
    /// @brief returns the top element of the queue and removes it.
    /// @return the top element of the queue.

    pair<T, int> element = this->queue_.front();
    this->queue_.pop_front();
    return element.first;
  }

  int size()
  {
    /// @brief get the number of queue_elements
    /// @return number of queue_elements

    return this->queue_.size();
  };

  template <class N>
  friend ostream &operator<<(ostream &os, const PriorityQueue<T> &queue);

private:
  /// @brief each queue element is a pair of the element and its current priority
  list<pair<T, int>> queue_;
};

template <class T>
ostream &operator<<(ostream &os, const PriorityQueue<T> &queue)
{
  /// @brief Output operator for PriorityQueue<T> class
  /// @tparam T Type of the queue elements
  /// @param os output stream
  /// @param queue the queue element to add to the output
  /// @return output stream

  os << "Queue with elements: " << endl;
  for (auto element : queue.queue_)
  {
    os << "Priority: " << element.second << " Data: " << element.first << endl;
  }
  return os;
}

class TreeNode
/// @brief class to hold information about a node in a graph and the node it can reach

{

public:
  TreeNode() {}

  TreeNode(const TreeNode &ele) : node_(ele.node_),
                                  destinations_(ele.destinations_) {}

  TreeNode(int node, const vector<TreeNode> &destinations) : node_(node),
                                                             destinations_(destinations) {}

  ~TreeNode() {}

  inline int node() const
  {
    /// @brief get the source node of this element
    /// @return node index

    return this->node_;
  }

  inline vector<TreeNode> destinations() const
  {
    /// @brief get the reachable nodes of this element
    /// @return list of node idx

    return this->destinations_;
  }

  friend ostream &operator<<(ostream &os, const TreeNode &set);

private:
  /// @brief source node
  int node_;
  /// @brief reachable nodes of this element
  vector<TreeNode> destinations_;
};

ostream &operator<<(ostream &os, const TreeNode &node)
{
  /// @brief Output operator for TreeNode class
  /// @param os output stream
  /// @param node the node element to add to the output
  /// @return output stream

  os << "(" << node.node_ << ",";
  for (auto dest : node.destinations_)
  {
    os << dest;
  }
  os << ")";
  return os;
}

template <class T>
class Graph
/// @brief Data type for a graph.

{
public:
  Graph() : vertices_(vector<T>(0)) {}

  Graph(const vector<T> &vertices) : vertices_(vertices)
  {
    for (int node = 0; node < this->vertices_.size(); node++)
    {
      this->edges_.insert(pair<int, map<int, int>>(node, map<int, int>()));
    }
  }

  Graph(const Graph<T> &graph) : vertices_(graph.vertices_), edges_(graph.edges_) {}

  Graph(const string filepath) : vertices_(vector<T>(0))
  {
    /// @brief Get graph from a file.  The file format will be an initial integer that is the
    ///        node size of the graph and the further values will be integer triples: (i, j, cost)
    /// @param filepath path to the file containing the graph definition
    ifstream graphfile(filepath);
    istream_iterator<T> start(graphfile), end;
    int size = static_cast<int>(*start);
    this->vertices_.resize(size);
    for (int node = 0; node < size; node++)
    {
      this->vertices_[node] = node;
      this->edges_.insert(pair<int, map<int, int>>(node, map<int, int>()));
    }
    vector<T> edges(++start, end);
    for (auto edge = edges.begin(); edge != edges.end(); edge = edge + 3)
    {
      int i = *edge;
      int j = *(edge + 1);
      int cost = *(edge + 2);
      this->addEdge(i, j);
      this->setEdgeValue(i, j, cost);
    }
  }

  ~Graph() {}

  inline vector<T> vertices()
  {
    /// @brief get all nodes of the graph
    /// @return all node values of the graph

    return vector<T>(vertices_);
  }

  int order() const
  {
    /// @brief get the order of the graph
    /// @return number of vertices of the graph

    return this->vertices_.size();
  }

  float density() const
  {
    /// @brief density of the graph
    /// @return density in the range [0..1]

    int maxEdges = this->order() * (this->order() - 1);
    int edgeCount = 0;
    for (auto entry : this->edges_)
    {
      edgeCount = edgeCount + entry.second.size();
    }
    return static_cast<float>(edgeCount) / maxEdges;
  }

  vector<int> neighbors(int x) const
  {
    /// @brief lists all nodes y such that there is an edge from x to y.
    /// @param x index of starting node of the edge
    /// @return all neighbors of the node

    vector<int> neighbors;
    if (0 <= x < this->order())
    {
      for (auto edge : this->edges_.at(x))
      {
        neighbors.push_back(edge.first);
      }
    }
    return neighbors;
  }

  T getNodeValue(int x) const
  {
    /// @brief returns the value associated with the node idx.
    /// @param x index of the node
    /// @return value of the node

    if (0 <= x < this->order())
    {
      return this->vertices_.at(x);
    }
    return NULL;
  }

  bool setNodeValue(int x, const T &node)
  {
    /// @brief sets the value associated with the node idx to a.
    /// @param x index of the node
    /// @param node new value of the node
    /// @return true if the node was successfully updated

    if (0 <= x < this->order())
    {
      this->vertices_.at(x) = node;
      return true;
    }
    return false;
  }

  bool addEdge(int x, int y)
  {
    /// @brief adds to the graph the edge from x to y, if it is not there.
    /// @param x index of start node of the edge
    /// @param y index of end node of the edge
    /// @return true if the edge was successfully added

    if (!this->adjacent(x, y) && 0 <= y < this->order())
    {
      this->edges_.at(x).insert(pair<int, int>(y, 1));
      return true;
    }
    return false;
  }

  bool deleteEdge(int x, int y)
  {
    /// @brief removes the edge from x to y, if it is there.
    /// @param x index of start node of the edge
    /// @param y index of end node of the edge
    /// @return true if the edge was successfully deleted

    if (this->adjacent(x, y))
    {
      this->edges_.at(x).erase(y);
    }
    return true;
  }

  int getEdgeValue(int x, int y) const
  {
    /// @brief returns the value associated to the edge (x,y).
    /// @param x index of start node of the edge
    /// @param y index of end node of the edge
    /// @return value of the edge

    if (this->adjacent(x, y))
    {
      return this->edges_.at(x).at(y);
    }
    return -1;
  }

  bool setEdgeValue(int x, int y, int v)
  {
    /// @brief sets the value associated to the edge (x,y) to v.
    /// @param x index of start node of the edge
    /// @param y index of end node of the edge
    /// @param v new value of the edge
    /// @return true if value is succefully updated

    if (this->adjacent(x, y))
    {
      this->edges_.at(x).at(y) = v;
      return true;
    }
    return false;
  }

  bool adjacent(int x, int y) const
  {
    /// @brief tests whether there is an edge from node x to node y.
    /// @param x index of start node
    /// @param y index of end node of the edge
    /// @return true if there is a edge

    return this->edges_.count(x) && this->edges_.at(x).count(y);
  }

  pair<int, TreeNode> getMinimumSpanningTree() const
  {
    /// @brief Prim's algorithm to get minimum spanning tree

    pair<int, TreeNode> result;
    result.first = -1;
    map<int, TreeNode> forest;
    PriorityQueue<pair<int, int>> sortedEdges;
    for (auto edge : this->edges_)
    {
      int source = edge.first;

      for (auto targetEdge : edge.second)
      {
        int target = targetEdge.first;
        int cost = targetEdge.second;
        sortedEdges.insert(pair(source, target), cost);
      }
    }

    return result;
  }

  template <class N>
  friend ostream &operator<<(ostream &os, const Graph<N> &graph);

private:
  /* All node values of the graph. The index in the vectoris used as a reference.
   */
  vector<T> vertices_;
  /*Edges are stored in a map to enablefast lookup if the starting node is given.
  - Key is the start node
  - All nodes that are the end point of a edge are stored in a end point map
      - Key is the index in the end node,
      - Value is the value of the edge
  */
  map<int, map<int, int>> edges_;
};

template <class T>
ostream &operator<<(ostream &os, const Graph<T> &graph)
{
  /// @brief Output operator for Graph<T> class
  /// @tparam T Type of the nodes of the graph
  /// @param os output stream
  /// @param graph the graph object to add to the output
  /// @return output stream

  os << "Graph of order " << graph.order();
  os << " with density " << graph.density() << endl;
  os << "- Nodes:" << endl;
  for (int node = 0; node < graph.order(); node++)
  {
    os << "  [Id: " << node << " Data: " << graph.getNodeValue(node) << "]";
    for (int neighbor : graph.neighbors(node))
    {
      os << " ->" << graph.getNodeValue(neighbor);
      os << "(" << graph.getEdgeValue(node, neighbor) << ") ";
    }
    os << endl;
  }
  os << endl;
  auto tree = graph.getMinimumSpanningTree();
  os << " Minimum Spanning Tree: Cost: " << tree.first << " Tree: ";
  os << tree.second << endl;
  return os;
}

class NodePath
/// @brief class to hold information about a node in a graph and path how it is reachable

{

public:
  NodePath() {}

  NodePath(const NodePath &ele) : node_(ele.node_),
                                  path_(ele.path_) {}

  NodePath(int node, const vector<int> &path) : node_(node),
                                                path_(path) {}

  ~NodePath() {}

  inline int node() const
  {
    /// @brief get the destination node of this element
    /// @return node index

    return this->node_;
  }

  inline vector<int> path() const
  {
    /// @brief get the current path to the node
    /// @return path as list of nodes

    return this->path_;
  }

  friend ostream &operator<<(ostream &os, const NodePath &set);
  friend bool operator==(const NodePath &set1, const NodePath &set2);
  friend bool operator!=(const NodePath &set1, const NodePath &set2);

private:
  /// @brief destination node
  int node_;
  /// @brief path to the destination node
  vector<int> path_;
};

bool operator==(const NodePath &set1, const NodePath &set2)
{
  /// @brief Compare operator for equality of two NodePath objects
  /// @param set1 NodePath object 1
  /// @param set2 NodePath object 2
  /// @return True if the destination nodes of both elements are equal

  return (set1.node() == set2.node());
}

bool operator!=(const NodePath &set1, const NodePath &set2)
{
  /// @brief Compare operator for non equality of two NodePath objects
  /// @param set1 NodePath object 1
  /// @param set2 NodePath object 2
  /// @return True if the destination nodes of both elements are not equal

  return (set1.node() != set2.node());
}

ostream &operator<<(ostream &os, const NodePath &set)
{
  /// @brief Output operator for NodePath class
  /// @param os output stream
  /// @param set the set element to add to the output
  /// @return output stream

  os << "Node: " << set.node() << endl;
  os << "Path: ";
  vector<int> path = set.path();
  for (int node : path)
  {
    os << node;
    if (node != path.at(path.size() - 1))
    {
      os << " -> ";
    }
  }
  os << endl;
  return os;
}

template <class T>
class ShortestPath
/// @brief implements the mechanics of Dijkstra’s algorithm

{

public:
  ShortestPath(const Graph<T> &g, int u, int w) : graph_(g),
                                                  start_(u),
                                                  destination_(w)
  {
    /// @param g the graph
    /// @param u index of start node
    /// @param w index of target node
    this->calculate();
  }

  ShortestPath(const ShortestPath<T> &short_path) : graph_(short_path.graph_),
                                                    start_(short_path.start_),
                                                    destination_(short_path.destination_),
                                                    path_(short_path.path_),
                                                    cost_(short_path.cost_) {}

  ShortestPath() : graph_(Graph<T>()),
                   start_(0),
                   destination_(0)
  {
    this->calculate();
  }

  ~ShortestPath() {}

  Graph<T> graph() const
  {
    /// @brief get the graph object of the path
    /// @return the graph object

    return this->graph_;
  }

  int start() const
  {
    /// @brief get the start node of the path
    /// @return the start node

    return this->start_;
  }

  int destination() const
  {
    /// @brief get the destination node of the path
    /// @return the destination node

    return this->destination_;
  }

  bool setStart(int u)
  {
    /// @brief set the start node of the path
    /// @param u index of node
    /// @return true if the update succeeded

    this->start_ = u;
    return this->calculate();
  }

  bool setDestination(int w)
  {
    /// @brief set the destination node of the path
    /// @param w index of node
    /// @return true if the update succeeded

    this->destination_ = w;
    return this->calculate();
  }

  vector<T> vertices()
  {
    /// @brief list of vertices in G(V,E).
    /// @return the vertices

    return this->graph_.vertices();
  }

  vector<int> path() const
  {
    /// @brief find shortest path between u-w
    /// @return the sequence of vertices representing shorest path u-v1-v2-…-vn-w.

    return this->path_;
  }

  int pathSize() const
  {
    /// @brief get the cost of the shortest path between u-w
    /// @return the path cost associated with the shortest path.

    return this->cost_;
  }

  template <class N>
  friend ostream &operator<<(ostream &os, const ShortestPath<N> &shortPath);

private:
  Graph<T> graph_;
  int start_;
  int destination_;
  vector<int> path_;
  int cost_;

  bool calculate()
  {
    /// @brief calculate the shortest path vector and the cost of this path
    /// @return True if calculation was succeeded

    PriorityQueue<NodePath> openSet;
    vector<bool> closedSet = vector<bool>(this->vertices().size(), false);
    int currentCost = 0;
    vector<int> currentPath = vector<int>{this->start_};
    int nextNode = this->start_;
    while (nextNode != destination_ && count(closedSet.begin(), closedSet.end(), false))
    {
      closedSet.at(nextNode) = true;
      for (int node : this->graph_.neighbors(nextNode))
      {
        if (closedSet.at(node))
        {
          continue;
        }
        int cost = this->graph_.getEdgeValue(nextNode, node);
        vector<int> nodePath = vector<int>(currentPath);
        nodePath.push_back(node);
        openSet.insert(NodePath(node, nodePath), cost + currentCost);
      }
      currentCost = openSet.minPrioirty();
      NodePath element = openSet.popTop();
      currentPath = element.path();
      nextNode = element.node();
    }
    if (nextNode == destination_)
    {
      this->cost_ = currentCost;
      this->path_ = currentPath;
      return true;
    }
    this->cost_ = -1;
    this->path_ = vector<int>();
    return false;
  }
};

template <class T>
ostream &operator<<(ostream &os, const ShortestPath<T> &shortPath)
{
  /// @brief Output operator for Graph<T> class
  /// @tparam T Type of the nodes of the graph
  /// @param os output stream
  /// @param shortPath the shortest path to add to the output
  /// @return output stream

  os << "Distance from " << shortPath.start() << " to ";
  os << shortPath.destination() << " is " << shortPath.pathSize() << endl;
  os << "Shortest path from " << shortPath.start() << " to ";
  os << shortPath.destination() << " is ";
  for (int node : shortPath.path())
  {
    os << node;
    if (node != shortPath.destination())
    {
      os << " -> ";
    }
  }
  os << endl;
  return os;
}

Graph<string> graph_generator(int nodecount, float density, unsigned distance_range)
{
  /// @brief generates a graph with the given number of nodes and
  ///        approximately the given density
  /// @param nodecount number of nodes
  /// @param density requested densityin the range [0..1]
  /// @param distance_range max distance value of an edge
  /// @return the generated graph object

  srand(static_cast<unsigned>(time(NULL)));
  vector<string> nodes;
  for (int i = 0; i < nodecount; i++)
  {
    nodes.push_back(to_string(i));
  }
  Graph<string> testgraph = Graph<string>(nodes);
  int size = testgraph.order();
  for (int i = 0; i < size; i++)
  {
    for (int j = 0; j < size; j++)
    {
      if (i != j && ((static_cast<double>(rand()) / RAND_MAX) < density))
      {
        testgraph.addEdge(i, j);
        testgraph.setEdgeValue(i, j, rand() % distance_range);
      }
    }
  }
  return testgraph;
}

int main()
{
  // Graph with 50 nodes and 20% density
  Graph<string> testgraph = graph_generator(50, 0.2, 10);
  cout << testgraph << endl;
  auto shortest_path = ShortestPath<string>(testgraph, 3, 33);
  cout << shortest_path << endl;
  shortest_path.setStart(34);
  cout << shortest_path << endl;

  // Graph with 50 nodes and 40% density
  testgraph = graph_generator(50, 0.4, 10);
  cout << testgraph << endl;
  shortest_path = ShortestPath<string>(testgraph, 1, 6);
  shortest_path.setDestination(45);
  cout << shortest_path << endl;

  // Graph from data file
  cout << "Graph from file ../data/SampleTestData_mst_data.txt" << endl;
  Graph<int> filegraph("../data/SampleTestData_mst_data.txt");
  cout << filegraph << endl;
  return 0;
}
