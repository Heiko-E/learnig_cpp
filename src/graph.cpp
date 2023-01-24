#include <iostream>
#include <vector>
using namespace std;

template <class T>
class Graph
/*
Abstract Data Type for a graph.
*/
{
public:
    Graph() : Vertices(vector<T>(0)) { ; };

    Graph(int size, float density) : Vertices(vector<T>(size))
    {
        vector<vector<int>> graph = vector<vector<int>>(size);
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                if (i == j)
                    graph[i][j] = false;
                else
                    graph[i][j] = graph[j][i] = (prob() < density);
            }
        }
    };

    ~Graph()
    {
        Vertices.clear();
        // delete[] Vertices;
    };

    vector<T> getVertices()
    {
        return Vertices;
    }

    vector<T, T> getEdges()
    {
        return NULL;
    }

    bool adjacent(const T &x, const T &y)
    {
        // tests whether there is an edge from node x to node y.
        return true;
    };

    vector<T> neighbors(const T &x)
    {
        // lists all nodes y such that there is an edge from x to y.
        return NULL;
    };

    bool add(const T &x, const T &y)
    {
        // adds to the graph the edge from x to y, if it is not there.
        return true;
    };

    bool deleteEdge(const T &x, const T &y)
    {
        // removes the edge from x to y, if it is there.
        return true;
    };

    int get_node_value(const T &x)
    {
        // returns the value associated with the node x.
        return true;
    };

    bool set_node_value(const T &x, int a)
    {
        // sets the value associated with the node x to a.
        return true;
    };

    int get_edge_value(const T &x, const T &y)
    {
        // returns the value associated to the edge (x,y).
        return 0;
    };

    bool set_edge_value(const T &x, const T &y, int v)
    {
        // sets the value associated to the edge (x,y) to v.
        return true;
    };

    friend ostream &operator<<(ostream &os, const Graph<T> &graph);

private:
    vector<T> Vertices;

    int prob()
    {
        return 1;
    }
};

template <class T>
ostream &operator<<(ostream &os, Graph<T> &graph)
{
    for (auto &&node : graph.getVertices())
    {
        os << node << ", ";
    }
    return os;
}

template <class T>
class PriorityQueue
/*
    The value of the PriorityQueue is to always have access to the vertex
    with the next shortest link in the shortest path calculation at the top of the queue.
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
// implements the mechanics of Dijkstra’s algorithm
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
    Graph<string> testgraph(4, 0.3);
    cout << testgraph;
    return 0;
}
