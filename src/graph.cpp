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

    inline bool adjacent(const T &x, const T &y)
    {
        /// @brief tests whether there is an edge from node x to node y.
        /// @param x index of start node
        /// @param y index of end node of the edge
        /// @return true if there is a edge
        return true;
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
        if (this->edges.count(x) && !this->edges.at(x).count(y) && 0 <= y < this->order())
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
        if (this->edges.count(x) && this->edges.at(x).count(y))
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
        if (this->edges.count(x) && this->edges.at(x).count(y))
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
        if (this->edges.count(x) && this->edges.at(x).count(y))
        {
            this->edges.at(x).at(y) = v;
            return true;
        }
        return false;
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

template <class T>
class PriorityQueue
/*
 * The value of the PriorityQueue is to always have access to the vertex
 * with the next shortest link in the shortest path calculation at the top of the queue.
 */
{
public:
    PriorityQueue() {}

    ~PriorityQueue() {}

    bool chgPrioirity(int priority)
    {
        // changes the priority (node value) of queue element.
        return false;
    }

    bool minPrioirty()
    {
        // removes the top element of the queue.
        return false;
    }

    bool contains(const T &queue_element)
    {
        // does the queue contain queue_element.
        return false;
    }

    bool Insert(const T &queue_element)
    {
        // insert queue_element into queue
        return false;
    }

    T top()
    {
        // returns the top element of the queue.
    }

    int size()
    {
        // return the number of queue_elements.
        return 0;
    };
};

template <class T>
class ShortestPath
/*
 * implements the mechanics of Dijkstra’s algorithm
 */
{
public:
    ShortestPath(const Graph<T> &g) : graph(g) {}

    ~ShortestPath() {}

    vector<T> vertices()
    {
        /// @brief list of vertices in G(V,E).
        /// @return the vertices
        return this->graph.getVertices();
    }

    vector<T> path(const T &u, const T &w)
    {
        // find shortest path between u-w and
        // returns the sequence of vertices representing shorest path u-v1-v2-…-vn-w.
        return NULL;
    }

    int path_size(const T &u, const T &w)
    {
        // return the path cost associated with the shortest path.
        return 0;
    }

private:
    Graph<T> graph;
};

Graph<string> graph_generator(int nodecount, float density)
{
    /// @brief generates a graph with the given number of nodes and
    ///        approximately the given density
    /// @param nodecount number of nodes
    /// @param density requested densityin the range [0..1]
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
            if (i != j && ((static_cast<double>(rand()) / RAND_MAX) <= density))
            {
                testgraph.addEdge(i, j);
                testgraph.addEdge(j, i);
                testgraph.set_edge_value(i, j, rand() % 100);
                testgraph.set_edge_value(j, i, rand() % 100);
            }
        }
    }
    return testgraph;
}

int main()
{
    Graph<string> testgraph = graph_generator(200, 0.01);
    cout << testgraph;
    ShortestPath<string> path = ShortestPath<string>(testgraph);

    return 0;
}
