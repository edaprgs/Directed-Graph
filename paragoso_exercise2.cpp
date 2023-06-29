/* 2021-1574 Eda Grace J. Paragoso
2021-0622 Nancy M. Sayson
2021-1853 Zeus Morley S. Pineda */

/* EXERCISE #2 output to the answers of Problem #3 and Bonus Question #2 */

#include <iostream>
#include <stack>
#include <queue>
#include <vector>
#include <limits>
#include <algorithm>

using namespace std;

const int INF = numeric_limits<int>::max();

struct Edge {
    int destination;
    int weight;
};

struct Vertex {
    int id;
    int distance;
    vector<int> path;

    Vertex(int _id, int _distance, const vector<int>& _path) : id(_id), distance(_distance), path(_path) {}
};

struct CompareVertex {
    bool operator()(const Vertex& v1, const Vertex& v2) {
        return v1.distance > v2.distance;
    }
};

class Graph {
private:
    vector<vector<Edge>> adjacencyList;

public:
    Graph(int numVertices) {
        adjacencyList.resize(numVertices + 1);  // Vertices are ordered in ascending order
    }

    void addEdge(int source, int destination, int weight) {
        Edge edge = {destination, weight};
        adjacencyList[source].push_back(edge);
    }

    void breadthFirstSpanningTree(int startVertex) {
        vector<bool> visited(adjacencyList.size(), false);
        queue<int> bfsQueue;

        visited[startVertex] = true;
        bfsQueue.push(startVertex);

        while (!bfsQueue.empty()) {
            int currentVertex = bfsQueue.front();
            bfsQueue.pop();

            cout << currentVertex << " ";

            for (const auto& edge : adjacencyList[currentVertex]) {
                int destination = edge.destination;
                if (!visited[destination]) {
                    visited[destination] = true;
                    bfsQueue.push(destination);
                }
            }
        }
    }

    void depthFirstSpanningTree(int startVertex) {
        vector<bool> visited(adjacencyList.size(), false);
        stack<int> dfsStack;

        visited[startVertex] = true;
        dfsStack.push(startVertex);

        while (!dfsStack.empty()) {
            int currentVertex = dfsStack.top();
            dfsStack.pop();

            cout << currentVertex << " ";

            for (const auto& edge : adjacencyList[currentVertex]) {
                int destination = edge.destination;
                if (!visited[destination]) {
                    visited[destination] = true;
                    dfsStack.push(destination);
                }
            }
        }
    }
};

// Dijkstra's algorithm to find the shortest paths
void dijkstra(const vector<vector<pair<int, int>>>& graph, int source) {
    int n = graph.size();

    // Priority queue to store vertices
    priority_queue<Vertex, vector<Vertex>, CompareVertex> pq;

    // Array to store distances from the source
    vector<int> distances(n + 1, INF);

    // Array to store paths
    vector<vector<int>> paths(n + 1);

    // Initialize the source vertex
    distances[source] = 0;
    paths[source].push_back(source);
    pq.push(Vertex(source, 0, paths[source]));

    while (!pq.empty()) {
        Vertex curr = pq.top();
        pq.pop();

        int u = curr.id;
        int dist = curr.distance;

        // If the current distance is greater than the stored distance, skip it
        if (dist > distances[u])
            continue;

        // Explore the neighbors of the current vertex
        for (const auto& neighbor : graph[u]) {
            int v = neighbor.first;
            int weight = neighbor.second;

            // If a shorter path is found, update the distance and path
            if (dist + weight < distances[v]) {
                distances[v] = dist + weight;
                paths[v] = curr.path;
                paths[v].push_back(v);
                pq.push(Vertex(v, dist + weight, paths[v]));
            }
        }
    }

    // Create a vector of vertices for sorting by distance
    vector<Vertex> sortedVertices;
    for (int i = 1; i <= n; ++i) {
        if (i == source || i == 11)  // Skip vertex 1 and 11
            continue;
        sortedVertices.push_back(Vertex(i, distances[i], paths[i]));
    }

    // Sort the vertices based on distance in ascending order
    sort(sortedVertices.begin(), sortedVertices.end(), [](const Vertex& v1, const Vertex& v2) {
        return v1.distance < v2.distance;
    });

    // Output the shortest paths in ascending order
    for (const auto& vertex : sortedVertices) {
        cout << "Shortest path from vertex " << source << " to " << vertex.id << " with path length = " << vertex.distance << ": ";
        if (vertex.distance == INF) {
            cout << "No path";
        } else {
            for (int j = 0; j < vertex.path.size(); ++j) {
                cout << vertex.path[j];
                if (j != vertex.path.size() - 1)
                    cout << ", ";
            }
        }
        cout << endl;
    }
}

int main() {

    cout << "EXERCISE #2 output to the answers of Problem #3 and Bonus Question #2" << endl;
    cout << endl;

    /* 3.A SHORTEST PATH */
    Graph graph(10);
    // Adding edges to the graph
    graph.addEdge(1, 4, 20);
    graph.addEdge(2, 5, 1);
    graph.addEdge(5, 3, 7);
    graph.addEdge(5, 8, 3);
    graph.addEdge(8, 6, 2);
    graph.addEdge(8, 10, 2);
    graph.addEdge(6, 3, 1);
    graph.addEdge(3, 1, 3);
    graph.addEdge(4, 3, 8);
    graph.addEdge(4, 6, 6);
    graph.addEdge(4, 7, 4);
    graph.addEdge(7, 8, 1);
    graph.addEdge(10, 9, 1);
    graph.addEdge(9, 7, 2);

    cout << "A. List the Shortest Paths" << endl;
    cout << "Shortest Paths using Dijkstra's Algorithm:" << endl;

    // Define the directed graph as an adjacency list
    vector<vector<pair<int, int>>> dijkstraGraph = {
        {},                                 // Vertex 0 (not used)
        { {2, 2}, {4, 20} },                // Vertex 1
        { {5, 1} },                         // Vertex 2
        { {1, 3} },                         // Vertex 3
        { {3, 8}, {6, 6}, {7, 4} },         // Vertex 4
        { {2, 7}, {8, 3} },                 // Vertex 5
        { {3, 1} },                         // Vertex 6
        { {8, 1} },                         // Vertex 7
        { {6, 2}, {10, 2} },                // Vertex 8
        { {7, 2}, {9, 1} },                 // Vertex 9
        { {9, 2} }                          // Vertex 10
    };
    dijkstra(dijkstraGraph, 1);  // Call Dijkstra's algorithm with source vertex 1

    /* 3.B QUESTION */
    cout << endl;
    cout << "B. Is the directed graph in Figure 1 connected? Why?" << endl;
    cout << "Yes, the directed graph above is connected because there is a path to traverse from every vertex to any other vertex in the graph." << endl;
    cout << endl;

    /* 3.C BREADTH-FIRST SPANNING TREE */
    cout << "C. Breadth-First Spanning Tree for Figure 1 starting from vertex 1: ";
    graph.breadthFirstSpanningTree(1);
    cout << endl;

    /* 3.D DEPTH-FIRST SPANNING TREE */
    cout << "D. Depth-First Spanning Tree for Figure 1 starting from vertex 1: ";
    graph.depthFirstSpanningTree(1);
    cout << endl;

    cout << endl;
    /* BONUS QUESTION #2 NUMBER OF EDGES */
    cout << "Bonus Question #2: Method/function to determine the number of edges in the graph";
    cout << endl;

    int edge_counter;
    int matrix[10][10] = {//This is the matrix of problem 3 which is the directed graph
        {0, 2, 0, 20, 0, 0, 0, 0, 0, 0},//1
        {0, 0, 0, 0, 1, 0, 0, 0, 0, 0},//2
        {3, 0, 0, 0, 0, 0, 0, 0, 0, 0},//3
        {0, 0, 0, 0, 0, 6, 4, 0, 0, 0},//4
        {0, 0, 0, 0, 0, 0, 0, 3, 0, 0},//5
        {0, 0, 1, 0, 0, 0, 0, 0, 0, 0},//6
        {0, 0, 0, 0, 0, 0, 0, 1, 0, 0},//7
        {0, 0, 0, 0, 0, 2, 0, 0, 0, 2},//8
        {0, 0, 0, 0, 0, 0, 2, 0, 0, 0},//9
        {0, 0, 0, 0, 0, 0, 0, 0, 1, 0}//10
    //   1  2  3  4  5  6  7  8  9  10
    };

    // Output the matrix
    cout << "The matrix:\n";
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            cout << matrix[i][j] << " ";
        }
        cout << endl;
    }

    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            if (matrix[i][j] > 0)
            {
                edge_counter++;
                matrix[i][j]=0;
                matrix[j][i]=0;
            }
        }
    }
    cout<< "Number of edges: "<<edge_counter;

    return 0;
}
