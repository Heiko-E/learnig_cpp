#include <iostream>
#include <vector>
using namespace std;

template <class T>
class Graph
/*
Abstract Data Type for a graph.
*/
{
    void Graph() : Vertices(vector<T>(0)){};
    void Graph(int size; int density) : Vertices(vector<T>(size))
    {
        bool[][] graph = bool[size][size];
        for (int i : = 0; i < size; i++)
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
    void ~Graph()
    {
        delete this->Vertices;
    };

public:
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

    bool delete(const T &x, const T &y)
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

private:
    vector<T> Vertices;
};

template <class T>
class PriorityQueue
{
    /*
    The value of the PriorityQueue is to always have access to the vertex
    with the next shortest link in the shortest path calculation at the top of the queue.
    */
    void PriorityQueue(){};
    void ~PriorityQueue(){};

public:
    bool chgPrioirity(int priority){
        // changes the priority (node value) of queue element.
    };

    bool minPrioirty(){
        // removes the top element of the queue.
    };

    bool contains(const T &queue_element){
        // does the queue contain queue_element.
    };

    bool Insert(const T &queue_element){
        // insert queue_element into queue
    };

    T top(){
        // returns the top element of the queue.
    };

    int size(){
        // return the number of queue_elements.
    };
};

template <class T>
class ShortestPath
{
    // implements the mechanics of Dijkstra’s algorithm
    void ShortestPath(Graph<T> g) : graph(g){};

    void ~ShortestPath()
    {
        delete this->graph;
    };

public:
    vector<T, T> vertices(List){
        // list of vertices in G(V,E).
    };

    vector<T> path(const T &u, const T &w){
        // find shortest path between u-w and
        // returns the sequence of vertices representing shorest path u-v1-v2-…-vn-w.
    };

    int path_size(const T &u, const T &w){
        // return the path cost associated with the shortest path.
    };

private:
    Graph<T> graph;
};

int main()
{
}
