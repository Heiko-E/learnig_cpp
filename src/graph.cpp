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
        // Copy constructor
        this->Vertices = vector<T>(graph.getVertices());
    };

    ~Graph()
    {
        Vertices.clear();
        // delete *Vertices;
    };

    inline vector<T> getVertices()
    {
        return vector<T>(Vertices);
    }

    inline vector<T, T> getEdges()
    {
        return vector<T, T>(0);
    }

    inline int order()
    {
        // get the order of the graph
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
        // tests whether there is an edge from node x to node y.
        return true;
    };

    inline vector<T> neighbors(const T &x)
    {
        // lists all nodes y such that there is an edge from x to y.
        return vector<T>(0);
    };

    inline T get_node_value(int idx)
    {
        // returns the value associated with the node idx.
        return this->Vertices.at(idx);
    };

    inline bool set_node_value(int idx, const T &a)
    {
        // sets the value associated with the node idx to a.
        this->Vertices.at(idx) = a;
        return true;
    };

    inline bool addEdge(const T &x, const T &y)
    {
        // adds to the graph the edge from x to y, if it is not there.
        return true;
    };

    inline bool deleteEdge(const T &x, const T &y)
    {
        // removes the edge from x to y, if it is there.
        return true;
    };

    inline int get_edge_value(const T &x, const T &y)
    {
        // returns the value associated to the edge (x,y).
        return 0;
    };

    inline bool set_edge_value(const T &x, const T &y, int v)
    {
        // sets the value associated to the edge (x,y) to v.
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
