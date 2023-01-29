#include <iostream>
#include <vector>
#include <map>
#include <bits/stdc++.h>
using namespace std;

template <class T>
class Graph
/*
 * Abstract Data Type for a graph.
 */
{
public:
    Graph() : vertices(vector<T>(0)) {}

    Graph(const vector<T> &vertices) : vertices(vertices)
    {
        for (int node = 0; node < this->vertices.size(); node++)
        {
            this->edges.insert(pair<int, map<int, int>>(node, map<int, int>()));
        }
    }

    Graph(const Graph<T> &graph) : vertices(graph.vertices), edges(graph.edges) {}

    ~Graph() {}

    inline vector<T> getVertices()
    {
        /// @brief get all nodes of the graph
        /// @return all node values of the graph
        return vector<T>(vertices);
    }

    inline int order()
    {
        /// @brief get the order of the graph
        /// @return number of vertices of the graph
        return this->vertices.size();
    }

    float density()
    {
        /// @brief density of the graph
        /// @return density in the range [0..1]
        int maxEdges = this->order() * (this->order() - 1);
        int edgeCount = 0;
        for (pair<int, map<int, int>> entry : this->edges)
        {
            edgeCount = edgeCount + entry.second.size();
        }
        return static_cast<float>(edgeCount) / maxEdges;
    }

    inline vector<int> neighbors(int x)
    {
        /// @brief lists all nodes y such that there is an edge from x to y.
        /// @param x index of starting node of the edge
        /// @return all neighbors of the node
        vector<int> neighbors;
        if (0 <= x < this->order())
        {
            for (pair<int, int> edge : this->edges.at(x))
            {
                neighbors.push_back(edge.first);
            }
        }
        return neighbors;
    }

    inline T get_node_value(int x)
    {
        /// @brief returns the value associated with the node idx.
        /// @param x index of the node
        /// @return value of the node
        if (0 <= x < this->order())
        {
            return this->vertices.at(x);
        }
        return NULL;
    }

    inline bool set_node_value(int x, const T &node)
    {
        /// @brief sets the value associated with the node idx to a.
        /// @param x index of the node
        /// @param node new value of the node
        /// @return true if the node was successfully updated
        if (0 <= x < this->order())
        {
            this->vertices.at(x) = node;
            return true;
        }
        return false;
    }

    inline bool addEdge(int x, int y)
    {
        /// @brief adds to the graph the edge from x to y, if it is not there.
        /// @param x index of start node of the edge
        /// @param y index of end node of the edge
        /// @return true if the edge was successfully added
        if (!this->adjacent(x, y) && 0 <= y < this->order())
        {
            this->edges.at(x).insert(pair<int, int>(y, 1));
            return true;
        }
        return false;
    }

    inline bool deleteEdge(int x, int y)
    {
        /// @brief removes the edge from x to y, if it is there.
        /// @param x index of start node of the edge
        /// @param y index of end node of the edge
        /// @return true if the edge was successfully deleted
        if (this->adjacent(x, y))
        {
            this->edges.at(x).erase(y);
        }
        return true;
    }

    inline int get_edge_value(int x, int y)
    {
        /// @brief returns the value associated to the edge (x,y).
        /// @param x index of start node of the edge
        /// @param y index of end node of the edge
        /// @return value of the edge
        if (this->adjacent(x, y))
        {
            return this->edges.at(x).at(y);
        }
        return -1;
    }

    inline bool set_edge_value(int x, int y, int v)
    {
        /// @brief sets the value associated to the edge (x,y) to v.
        /// @param x index of start node of the edge
        /// @param y index of end node of the edge
        /// @param v new value of the edge
        /// @return true if value is succefully updated
        if (this->adjacent(x, y))
        {
            this->edges.at(x).at(y) = v;
            return true;
        }
        return false;
    }

    inline bool adjacent(int x, int y)
    {
        /// @brief tests whether there is an edge from node x to node y.
        /// @param x index of start node
        /// @param y index of end node of the edge
        /// @return true if there is a edge
        return this->edges.count(x) && this->edges.at(x).count(y);
    }

    template <class N>
    friend ostream &operator<<(ostream &os, const Graph<N> &graph);

private:
    /* All node values of the graph. The index in the vectoris used as a reference.
     */
    vector<T> vertices;
    /*Edges are stored in a map to enablefast lookup if the starting node is given.
    - Key is the start node
    - All nodes that arre the end point of a edge are stored in a end point map
        - Key is the index in the end node,
        - Value is the value of the edge
    */
    map<int, map<int, int>> edges;
};

template <class T>
ostream &operator<<(ostream &os, Graph<T> &graph)
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
        os << "  " << node << ". " << graph.get_node_value(node) << ":";
        for (int neighbor : graph.neighbors(node))
        {
            os << " ->" << graph.get_node_value(neighbor);
            os << "(" << graph.get_edge_value(node, neighbor) << ") ";
        }
        os << endl;
    }
    os << endl;
    return os;
}

class PriorityQueue
/*
 * The value of the PriorityQueue is to always have access to the vertex
 * with the next shortest link in the shortest path calculation at the top of the queue.
 */
{
public:
    PriorityQueue() {}

    ~PriorityQueue() {}

    bool chgPrioirity(int queue_element, int priority)
    {
        /// @brief changes the priority (node value) of queue element.
        /// @param queue_element the  element to change
        /// @param priority the new priority
        /// @return true if the update succeeded
        return false;
    }

    int minPrioirty()
    {
        /// @brief priority of the top element of the queue.
        /// @return the pririty of the top element.
        return 0;
    }

    bool contains(int queue_element)
    {
        /// @brief does the queue contain the element.
        /// @param queue_element the node to look for
        /// @return true if node is in the queue
        return false;
    }

    bool Insert(int queue_element, int priority)
    {
        /// @brief insert node into queue
        /// @param queue_element the node to insert
        /// @param priority the priority of the node
        /// @return true if the insertion succeeded
        if (this->contains(queue_element))
        {
            this->queue.push_back(queue_element);
        }
        else
        {
            this->chgPrioirity(queue_element, priority);
        }
        return true;
    }

    int top()
    {
        /// @brief returns the top element of the queue and removes it.
        /// @return the top element of the queue.
        return -1;
    }

    int size()
    {
        /// @brief get the number of queue_elements
        /// @return number of queue_elements
        return this->queue.size();
    };

private:
    vector<int> queue;
};

template <class T>
class ShortestPath
/*
 * implements the mechanics of Dijkstra’s algorithm
 */
{
public:
    ShortestPath(const Graph<T> &g, int u, int w) : graph(g),
                                                    start(u),
                                                    destination(w)
    {
        /// @param g the graph
        /// @param u index of start node
        /// @param w index of target node
        this->calculate();
    }

    ShortestPath(const ShortestPath<T> &short_path) : graph(short_path.graph),
                                                      start(short_path.start),
                                                      destination(short_path.destination),
                                                      short_path(short_path.short_path),
                                                      cost(short_path.cost) {}

    ShortestPath() : graph(Graph<T>()),
                     start(0),
                     destination(0)
    {
        this->calculate();
    }

    ~ShortestPath() {}

    Graph<T> getGraph()
    {
        /// @brief get the graph object of the path
        /// @return the graph object
        return this->graph;
    }

    int getStart()
    {
        /// @brief get the start node of the path
        /// @return the start node
        return this->start;
    }

    int getDestination()
    {
        /// @brief get the destination node of the path
        /// @return the destination node
        return this->destination;
    }

    bool setStart(int u)
    {
        /// @brief set the start node of the path
        /// @param u index of node
        /// @return true if the update succeeded
        this->start = u;
        return this->calculate();
    }

    bool setDestination(int w)
    {
        /// @brief set the destination node of the path
        /// @param w index of node
        /// @return true if the update succeeded
        this->destination = w;
        return this->calculate();
    }

    vector<T> vertices()
    {
        /// @brief list of vertices in G(V,E).
        /// @return the vertices
        return this->graph.getVertices();
    }

    vector<int> path()
    {
        /// @brief find shortest path between u-w
        /// @return the sequence of vertices representing shorest path u-v1-v2-…-vn-w.
        return this->short_path;
    }

    int path_size()
    {
        /// @brief get the cost of the shortest path between u-w
        /// @return the path cost associated with the shortest path.
        return this->cost;
    }

    template <class N>
    friend ostream &operator<<(ostream &os, const ShortestPath<N> &short_path);

private:
    Graph<T> graph;
    int start;
    int destination;
    vector<int> short_path;
    int cost;

    bool calculate()
    {
        PriorityQueue open_set;
        vector<bool> closed_set = vector<bool>(this->vertices().size(), false);
        closed_set.at(this->start) = true;
        int next = this->start;
        int current_cost = 0;
        while (next != destination && count(closed_set.begin(), closed_set.end(), this->destination))
        {
            for (int node : this->graph.neighbors(next))
            {
                int cost = this->graph.get_edge_value(start, next);
                open_set.Insert(node, cost + current_cost);
            }
            current_cost = open_set.minPrioirty();
            next = open_set.top();
        }
        return true;
    }
};

template <class T>
ostream &operator<<(ostream &os, ShortestPath<T> &short_path)
{
    /// @brief Output operator for Graph<T> class
    /// @tparam T Type of the nodes of the graph
    /// @param os output stream
    /// @param short_path the shortest path to add to the output
    /// @return output stream
    os << "Distance from " << short_path.getStart() << " to ";
    os << short_path.getDestination() << " is " << short_path.path_size() << endl;
    os << "Path from " << short_path.getStart() << " to ";
    os << short_path.getDestination() << " is ";
    for (int node : short_path.path())
    {
        os << node << ". " << short_path.getGraph().get_node_value(node);
        if (node != short_path.getDestination())
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
                testgraph.addEdge(j, i);
                testgraph.set_edge_value(i, j, rand() % distance_range);
                testgraph.set_edge_value(j, i, rand() % distance_range);
            }
        }
    }
    return testgraph;
}

int main()
{
    Graph<string> testgraph = graph_generator(200, 0.01, 10);
    cout << testgraph << endl;
    ShortestPath<string> shortest_path = ShortestPath<string>(testgraph, 3, 65);
    cout << shortest_path << endl;
    return 0;
}
