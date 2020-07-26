#include <vector>
#include <queue>
#include <stack>
#include <unordered_set>
#include <unordered_map>
#include <set>
#include <deque>
#include <map>
#include <cstddef>
#include <string>
#include <utility>
#include <algorithm>
#include <climits>
#include <optional>
#include "directed_graph.hpp"

using namespace std;

/*
 * Computes the shortest distance from u to v in graph g.
 * The shortest path corresponds to a sequence of vertices starting from u and ends at v,
 * which has the smallest total weight of edges among all possible paths from u to v.
 */
template < typename T >
  vector < vertex < T >> shortest_path(directed_graph < T > & g, int u_id, int v_id) {

    vector < vertex < T >> shortest_city_path; //contains vertices from u to v in the shortest path order
    priority_queue < pair < int, int > , vector < pair < int, int >> , greater < pair < T, int >>> to_visit; //
    int tot_verticies = g.num_vertices(); //contains the total number of vertices
    vector < int > dist(tot_verticies); //tracks dist value to for picking min edge weight
    vector < int > prev(tot_verticies); //tracks the previous/parent vertex
    auto adj_list = g.get_adj_list(); //contains the adj_list from direct_graph.hpp

    for (auto vert: g.get_vertices()) { //initialise dist to max and prev to -1 for all vertices
      dist[vert.id] = INT_MAX;
      prev[vert.id] = -1;
    }

    to_visit.push(make_pair(0, u_id)); //push the source/root vertex
    dist[u_id] = 0; //initialise the distance of source/root vertex to 0 as no travel has been done

    while (!to_visit.empty()) { //loop until the priority queue is empty
      int u = to_visit.top().second; //make u equal to the weight of the vertex at the top of the priority queue
      to_visit.pop(); //pops the top vertex

      /*
       *  When destination vertex is reached insert vertices until source/root vertex is inserted    
       */
      if (u == v_id) { //checks if destination has been reached
        int next = v_id;
        if (prev[next] != -1 || next == u_id) { //if the previous vertex has been visited or the current vertex is equal to the root, while the vertex has been visited
          while (next != -1) {
            shortest_city_path.insert(shortest_city_path.begin(), vertex(next, g.get_weight(next))); //insert the current vertex to the front of shortest_city_path
            next = prev[next]; //make the next vertex to insert equal to the previous vertex of the current vertex
          }
        }
      }

      /*
       *  pushes the smallest cost (edge weight) to vertex v into the priority queue
       */
      for (auto x: adj_list[u]) { //for each edge with source vertex u
        int v = x.first; //initialise v as the end vertex of the edge
        int weight = x.second; //intialise weight as the edge weight of this edge

        if (dist[v] > dist[u] + weight) { //if the current cost to vertex v is more than the cost from the root to vertex u + the cost from vertex u to vertex v
          dist[v] = dist[u] + weight; //make cost to vertex v the cost of this new path 
          prev[v] = u; //set the previous vertex of vertex v to vertex u
          to_visit.push(make_pair(dist[v], v)); //push this new cost for vertex v into the priority queue
        }
      }
    }
    return shortest_city_path; //returns the shortest vertex path from vertex u to v
  }

template < typename T >
  vector < vector < vertex < T >>> strongly_connected_components(directed_graph < T > & g) {

    vector < vector < vertex < T >>> scc_list; //contains all the scc in seperate vectors
    int tot_verticies = g.num_vertices(); //contains the total number of vertices
    int * disc = new int[tot_verticies + 1]; //stores discovery time of each vertex
    int * low = new int[tot_verticies + 1]; //stores the vertex with the smallest discovery time that can be reached from 
    bool * in_stack = new bool[tot_verticies]; //tracks when a vertex is in the stack
    stack < int > * curr_stack = new stack < int > (); //stack for tracking ancestors when performing dfs

    for (auto vert: g.get_vertices()) { //initialises disc and low at -1 for all vertices, and makes sure no vertices are in the stack currently
      disc[vert.id] = -1;
      low[vert.id] = -1;
      in_stack[vert.id] = false;
    }

    for (auto vert: g.get_vertices()) {
      if (disc[vert.id] == -1) { //makes sure the vertex hasn't been discovered yet
        tarjan_DFS(vert.id, disc, low, curr_stack, in_stack, g, scc_list); //performs a dfs using the current vertex as the source and pushing any scc into the scc_list
      }
    }
    return scc_list; //returns a vector than contains a vector of vertices for each scc 
  }

template < typename T >
  void tarjan_DFS(int u, int disc[], int low[], stack < int > * curr_stack, bool in_stack[], directed_graph < T > g, vector < vector < vertex < T >>> & scc_list) {

    static int time = 0; //sets current discovery time to 0
    curr_stack -> push(u); //pushes the source vertex into the stack
    in_stack[u] = true;
    disc[u] = low[u] = ++time; //initialises  the discovery time for the current vertex

    for (auto n: g.get_neighbours(u)) { //for each neighbour of vertex u
      int v = n.id;

      if (disc[v] == -1) { //if vertex v hasn't been discovered perform dfs and then set the lowest discovery time of vertex u to the minimum between vertex u and vertex v
        tarjan_DFS(v, disc, low, curr_stack, in_stack, g, scc_list);
        low[u] = min(low[u], low[v]);
      } else if (in_stack[v] == true) { //if vertex v is already in the stack it is an ancestor, so make the lowest discovery time of vertex u to the minimum between vertex u and vertex v
        low[u] = min(low[u], low[v]);
      }
    }

    /*
     *  if scc are found push each set of scc into its own vector, then push each vector into scc_list
     */
    vector < vertex < T >> scc;
    if (low[u] == disc[u]) { //if the lowest discovery time and the discovery time of vertex u is equal then the vertices are scc
      while (curr_stack -> top() != u) { //push the top component of the stack into vector scc until the top vertex is vertex u
        scc.push_back(vertex(curr_stack -> top(), g.get_weight(curr_stack -> top())));
        in_stack[curr_stack -> top()] = false;
        curr_stack -> pop();
      } //then push vertex u into vector scc
      scc.push_back(vertex(curr_stack -> top(), g.get_weight(curr_stack -> top())));
      in_stack[curr_stack -> top()] = false;
      curr_stack -> pop();
      scc_list.push_back(scc); //then push vector scc into the scc_list
    }
  }

template < typename T >
  vector < vertex < T >> topological_sort(directed_graph < T > g) {

    vector < vertex < T >> topo_order; //contains the vertices of the graph in topological order
    int tot_verticies = g.num_vertices(); //contains the total number of vertices 
    bool * visited = new bool[tot_verticies]; //tracks when a vertex is visited

    for (auto vert: g.get_vertices()) { //marks all vertices as not visited
      visited[vert.id] = false;
    }

    for (auto vert: g.get_vertices()) { //for each vertex that hasn't been visited perform a DFS, pushing vertices into topo_order in topological order
      if (visited[vert.id] == false) {
        topo_DFS(vert.id, visited, topo_order, g);
      }
    }
    return topo_order; //returns the vertices of graph g in topological order
  }

template < typename T >
  void topo_DFS(int u, bool visited[], vector < vertex < T >> & topo_order, directed_graph < T > g) {

    visited[u] = true; //marks the source vertex as visited

    for (auto n: g.get_neighbours(u)) { //performs a dfs
      if (!visited[n.id]) {
        topo_DFS(n.id, visited, topo_order, g);
      }
    }
    topo_order.insert(topo_order.begin(), vertex(u, g.get_weight(u))); //inserts the current vertex to the front of topo_order
  }

/*
 * Computes the lowest cost-per-person for delivery over the graph.
 * u is the source vertex, which send deliveries to all other vertices.
 * vertices denote cities; vertex weights denote cities' population;
 * edge weights denote the fixed delivery cost between cities, which is irrelevant to
 * the amount of goods being delivered.
 */
template < typename T >
  T low_cost_delivery(directed_graph < T > g, int u_id) {
    // vertex = city
    // vertex weight = city population
    // edge = fixed delivery cost between cities
    directed_graph<T> mst = g.out_tree(u_id); // 1. create a minimum spanning tree

    int edges = 0, population = 0; // 2. initalise edges & population
    
    for(auto& vert : mst.get_vertices()){
        if(vert.id != u_id) { // do not count the starting vert
            // cout << vert.id <<  " population: " << vert.weight << endl;
            population += vert.weight; // 3. add all vertices in the mst except for the starting vert
        }
        for(auto& n_vert : mst.get_neighbours(vert.id)){
            // cout << vert.id << " -> "  << n_vert.id << " = " << mst.get_edge_weight(vert.id,n_vert.id) << endl;
            edges += mst.get_edge_weight(vert.id, n_vert.id); // 4. add all edges in mst 
        }
    }
    
    double total = edges/population; // 5. calculate the minimum cost per person

    // cout << "total population: " << population << endl;
    // cout << "edge weight / population = " << total << endl;
    // cout << edges << "/" << population << " = " << total << endl;

    return total; // return the minimum
  
}
