#include <iostream>
#include <vector>
using namespace std;

template <class T>
class Graph
/*
 * Abstract Data Type for a graph.
 */
{
public:
    Graph() : Vertices(vector<T>(0)){};

    Graph(const vector<T> &vertices) : Vertices(vertices){};

    Graph(const Graph<T> &graph)
    {
        /// @brief copy constructor
        this->Vertices = vector<T>(graph.getVertices());
    };

    ~Graph()
    {
        Vertices.clear();
        // delete *Vertices;
    };

    inline vector<T> getVertices()
    {
        /// @brief get all nodes of the graph
        /// @return all node values of the graph
        return vector<T>(Vertices);
    }

    inline vector<T, T> getEdges()
    {
        /// @brief get all edges of the graph as list
        /// @return all edges in the graph
        return vector<T, T>(0);
    }

    inline int order()
    {
        /// @brief get the order of the graph
        /// @return number of vertices of the graph
        return this->Vertices.size();
    };

    float density()
    {
        /// @brief density of the graph
        /// @return density in the range [0..1]
        return 0;
    };

    inline bool adjacent(const T &x, const T &y)
    {
        /// @brief tests whether there is an edge from node x to node y.
        /// @param x start node
        /// @param y end node of the edge
        /// @return true if there is a edge
        return true;
    };

    inline vector<T> neighbors(const T &x)
    {
        /// @brief lists all nodes y such that there is an edge from x to y.
        /// @param x starting node of the edge
        /// @return all neighbors of the node
        return vector<T>(0);
    };

    inline T get_node_value(int idx)
    {
        /// @brief returns the value associated with the node idx.
        /// @param idx number of the node
        /// @return value of the node
        return this->Vertices.at(idx);
    };

    inline bool set_node_value(int idx, const T &node)
    {
        /// @brief sets the value associated with the node idx to a.
        /// @param idx number of the node
        /// @param node new value of the node
        /// @return true if the node was successfully updated
        this->Vertices.at(idx) = node;
        return true;
    };

    inline bool addEdge(const T &x, const T &y)
    {
        /// @brief adds to the graph the edge from x to y, if it is not there.
        /// @param x start node of the edge
        /// @param y end node of the edge
        /// @return true if the edge was successfully added
        return true;
    };

    inline bool deleteEdge(const T &x, const T &y)
    {
        /// @brief removes the edge from x to y, if it is there.
        /// @param x start node of the edge
        /// @param y end node of the edge
        /// @return true if the edge was successfully deleted
        return true;
    };

    inline int get_edge_value(const T &x, const T &y)
    {
        /// @brief returns the value associated to the edge (x,y).
        /// @param x start node of the edge
        /// @param y end node of the edge
        /// @return value of the edge
        return 0;
    };

    inline bool set_edge_value(const T &x, const T &y, int v)
    {
        /// @brief sets the value associated to the edge (x,y) to v.
        /// @param x start node of the edge
        /// @param y end node of the edge
        /// @param v new value of the edge
        /// @return true if value is succefully updated
        return true;
    };

    template <class N>
    friend ostream &operator<<(ostream &os, const Graph<N> &graph);

private:
    vector<T> Vertices;
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
    for (T node : graph.getVertices())
    {
        os << "  - " << node << ":";
        for (T neighbor : graph.neighbors(node))
        {
            os << " ->" << neighbor << "(" << graph.get_edge_value(node, neighbor) << ") ";
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
    PriorityQueue(){};

    ~PriorityQueue(){};

    bool chgPrioirity(int priority)
    {
        // changes the priority (node value) of queue element.
        return false;
    };

    bool minPrioirty()
    {
        // removes the top element of the queue.
        return false;
    };

    bool contains(const T &queue_element)
    {
        // does the queue contain queue_element.
        return false;
    };

    bool Insert(const T &queue_element)
    {
        // insert queue_element into queue
        return false;
    };

    T top(){
        // returns the top element of the queue.
    };

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
    ShortestPath(Graph<T> g) : graph(g){};

    ~ShortestPath()
    {
        delete this->graph;
    };

    vector<T, T> vertices()
    {
        // list of vertices in G(V,E).
        return vector<T, T>(0);
    };

    vector<T> path(const T &u, const T &w)
    {
        // find shortest path between u-w and
        // returns the sequence of vertices representing shorest path u-v1-v2-…-vn-w.
        return NULL;
    };

    int path_size(const T &u, const T &w)
    {
        // return the path cost associated with the shortest path.
        return 0;
    };

private:
    Graph<T> graph;
};

int main()
{
    const float density = 0.1; // Density in the range [0..1]
    srand((unsigned)time(NULL));
    Graph<string> testgraph(vector<string>{"one", "two", "three", "four"});
    int size = testgraph.order();
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < size; j++)
        {
            if (i != j && ((static_cast<double>(rand()) / RAND_MAX) <= density))
            {
                string node1 = testgraph.get_node_value(i);
                string node2 = testgraph.get_node_value(j);
                testgraph.addEdge(node1, node2);
                testgraph.addEdge(node2, node1);
            }
        }
    }
    cout << testgraph;
    return 0;
}
