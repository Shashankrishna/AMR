/* Robot Traversal starting at any point in the given workspace, based on Graph theory.
Node = Vertices = Corners of room
Edges = Links = Traversal path */


#include <iostream>     
#include <vector>
#include <cstring>
#include <unordered_map>
#include <queue>
#include <list>

using namespace std;

// Define a class Graph: Directed graph using adjacency list representation.
// 1> An unordered list is used to represent the graph.
// 2> Define Breadth First Search algorithm

class Graph
{
public:
    int N;                                 // Number of Nodes
        
    list<int> *adj;                        // Pointer to array containing adjacency lists
 
    Graph(int);                            // Constructor called when object of the defined class is defined.
 
    void addEdge(int, int);                // Add edges (connections) between nodes 
  
    vector<int> BFS(int, int, int []);     // BFS Algorithm
};
 
Graph::Graph(int N)                       
{
    this->N = N;                          // Refering to the Nodes (instance variable) of the class Graph
    adj = new list<int>[N+1];             // Creating adjacency list
}
 
void Graph::addEdge(int a, int b)                
{
    adj[a].push_back(b);                         // An edge will have two ends, so it is referred as 'a' and 'b'
    adj[b].push_back(a);                         // Add a to b's list and vice versa.           
}
 
vector<int> Graph::BFS(int componentNum, int src, int visited[])
{
    // Initially consider the nodes are not visited.
    // Create a queue. [FIFO Method]
    queue<int> queue;
 
    queue.push(src);                   // Push source node to queue
 
    visited[src] = componentNum;      // Assign component number for visited node
 
    vector<int> reachableNodes;      // Vector to store all the reachable nodes from source node
 
    while(!queue.empty())                 // Condition: check if the queue is not empty and proceed
    {
        // Dequeue a node from queue
        int a = queue.front();           // Node is dequeued i.e., popped out and added to visited list.
        queue.pop();
 
        reachableNodes.push_back(a);
 
        // Get all adjacent nodes of the dequeued node 'a'. 
        // If an adjacent node has not been visited yet, then mark it as visited and it will be updated to visited list.
        for (auto itr = adj[a].begin(); itr != adj[a].end(); itr++)  // Fixed iteration check on adjacency list.
        {
            if (!visited[*itr])
            {
                visited[*itr] = componentNum;    // Assign Component Number to all the reachable nodes
                queue.push(*itr);
            }
        }
    }
    return reachableNodes;
}
 
// Display all the Reachable Nodes from a node 'n'
void displayReachableNodes(int n, unordered_map <int, vector<int> > m)
{
    vector<int> temp = m[n];
    for (int i=0; i<temp.size(); i++)
        cout << temp[i] << " "; 
    cout << endl;
}
 
// Find the reachable nodes for every element in the array.
void findReachableNodes(Graph g, int array[], int n)
{

    int N = g.N;        // Number of nodes 
 
    
    int visited[N+1];      // Take an integer visited array and initialize all the elements with 0
    memset(visited, 0, sizeof(visited));   
 
    unordered_map <int, vector<int> > m;      // Map: Store list of reachable Nodes for the given node.
 
    // As we proceed further the component numbers of nodes for two rooms will change.
    // Component number will be '1' and '2'.
    int componentNum = 0;         // Initialize component Number with 0
 
    for (int i = 0 ; i < n ; i++)    // For each node in array[] find reachable nodes
    {
        int a = array[i];

        if (!visited[a])
        {
            componentNum++;   // component number increement
 
            // Store the reachable Nodes corresponding to the node 'i'
            m[visited[a]] = g.BFS(componentNum, a, visited);
        }
 
        // Reachable nodes from a achieved, print them by doing a look up from the map
        cout << "Reachable Nodes from " << a <<" are\n";
        displayReachableNodes(visited[a], m);
    }
}
 
// Test Functions: Main Program
int main()
{
    // Creating the graph of 7 nodes and two different rooms.
    int N = 7;   
    Graph g(N);

    // Square block: 
    g.addEdge(1, 2);       // Adding edges (links)  between node 1 and node 2.
    g.addEdge(2, 3);       // Adding edges (links)  between node 2 and node 3.
    g.addEdge(3, 4);       // Adding edges (links)  between node 3 and node 4.
    g.addEdge(4, 1);       // Adding edges (links)  between node 4 and node 1.

    // Additional Test:
    // g.addEdge(2, 6);       // Adding edges (links)  between node 2 and node 6.
    // g.addEdge(3, 6);       // Adding edges (links)  between node 3 and node 6.
    // g.addEdge(4, 5);       // Adding edges (links)  between node 4 and node 5.
    // g.addEdge(5, 6);       // Adding edges (links)  between node 5 and node 6.

    // Triangle:
    g.addEdge(5, 6);       // Adding edges (links)  between node 5 and node 6.
    g.addEdge(6, 7);       // Adding edges (links)  between node 6 and node 7.
    g.addEdge(7, 5);       // Adding edges (links)  between node 7 and node 5.

    // For every ith element in the array
    // Find all reachable nodes from chosen nodes
    int array[] = {1,6};
 
    // Find number of elements 
    int n = sizeof(array)/sizeof(int);
 
    findReachableNodes(g, array, n);  // Function call
 
    return 0;
}