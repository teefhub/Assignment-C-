#ifndef DIRECTED_GRAPH_H
#define DIRECTED_GRAPH_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <queue>
#include <stack>
#include <limits>
#include <utility> 
#include <algorithm>
#include <string>

using namespace std;

template <typename T>
class vertex {
	
public:
	int id;
	T weight;

	vertex(int v_id, T v_weight) : id(v_id), weight(v_weight) {}
};

template <typename T>
class directed_graph {

private:
	unordered_map<int, T> vertex_weights; // each element is a pair<id, weight> for a vertex
	unordered_map<int, unordered_map<int, T>> adj_list; // each element is a pair<vertex A, its neighbour B and (A,B)'s weight>
														// each neighbour is also a pair<B, weight of edge (A,B)>

public:
	unordered_map<int, T> get_vertex_weights(){
		return vertex_weights;
	}
	unordered_map<int, unordered_map<int, T>> get_adj_list(){
		return adj_list;
	}

	directed_graph(); //A constructor for directed_graph. The graph should start empty.
	~directed_graph(); //A destructor. Depending on how you do things, this may not be necessary.

	/* utility */

	static bool find_id_in_vector(const int& u_id, const vector<vertex<T>>& v){
		for(vertex<T> y: v){
			if(y.id==u_id){
				return true;
			}
		}
		return false;
	}

	static bool comp(const pair<int,T>& a, const pair<int,T>& b) {
    	return a.second > b.second;
	}

	/* handling vertex */
	
	void add_vertex(const vertex<T>&); //Adds the passed in vertex to the graph (with no edges).
	bool contains(const int&); //Returns true if the graph contains the given vertex_id, false 
	size_t num_vertices() const; //Returns the total number of vertices in the graph.
	vector<vertex<T>> get_vertices(); //Returns a vector containing all the vertices.
	void remove_vertex(const int&); //Removes the given vertex. Should also clear any incident edges.otherwise.
	
	/* handling edge */

	void add_edge(const int&, const int&, const T&); //Adds a weighted edge from the first vertex to the second.
	size_t num_edges() const; //Returns the total number of edges in the graph.
	vector<pair<vertex<T>,vertex<T>>> get_edges(); //Returns a vector containing all the edges.
	bool adjacent(const int&, const int&); //Returns true if the first vertex is adjacent to the second, false otherwise.
	vector<vertex<T>> get_neighbours(const int&); //Returns a vector of all vertices reachable from the given vertex except itself.
	vector<vertex<T>> get_second_order_neighbours(const int&); // Returns a vector of all second_order_neighbours.
	size_t in_degree(const int&); //Returns number of edges coming in to a vertex.
	size_t out_degree(const int&); //Returns the number of edges leaving a vertex.
	size_t degree(const int&); //Returns the degree of the vertex (both in edges and out edges).
	void remove_edge(const int&, const int&); //Removes the edge between the two vertices, if it exists.

	/* graph operatons */

	vector<vertex<T>> depth_first(const int&); //Returns vertices in depth-first traversal order from the given vertex.
	vector<vertex<T>> breadth_first(const int&); //Returns vertices in breadth-first traversal order from the given vertex.
	bool reachable(const int&, const int&); //Returns true if the second vertex is reachable from the first; false otherwise.
	bool contain_cycles(); // Return true if there exists a path from a vertex to itself; false otherwise.

	/* tree operatons */	
	directed_graph<T> out_tree(const int&); //Returns a spanning tree of the graph where every vertex is reachable from root.

	vector<vertex<T>> pre_order_traversal(const int&, directed_graph<T>&); // returns a vertice sequence out of a pre-order traversal from the given vertex.
	vector<vertex<T>> in_order_traversal(const int&, directed_graph<T>&); // returns a vertice sequence out of an in_order_traversal from the given vertex.
	vector<vertex<T>> post_order_traversal(const int&, directed_graph<T>&); // returns a vertice sequence out of a post_order_traversal from the given vertex.

	/* open question */
	vector<vertex<T>> significance_sorting(); // Return sorted vertices in descending order of their significance.
};

template <typename T>
directed_graph<T>::directed_graph() {}

template <typename T>
directed_graph<T>::~directed_graph() {}

/*
	part 1: add, check, remove, retrieve vertex
*/

template <typename T>
void directed_graph<T>::add_vertex(const vertex<T>& u) {

	if(!contains(u.id)){
		vertex_weights.insert({u.id, u.weight}); // step 1: add to all_vertices
		adj_list[u.id]=unordered_map<int, T>(); // step 2: add to adj_list
	}

}

template <typename T>
bool directed_graph<T>::contains(const int& u_id) {

	if(vertex_weights.find(u_id)!=vertex_weights.end()){
		return true;
	}
	return false;

}

template <typename T>
size_t directed_graph<T>::num_vertices() const {

	return vertex_weights.size();

}

template <typename T>
vector<vertex<T>> directed_graph<T>::get_vertices() {

	vector<vertex<T>> v;
	for(auto x: vertex_weights){
		v.push_back(vertex<T>(x.first, x.second));
	}
	return v;

}

template <typename T>
void directed_graph<T>::remove_vertex(const int& u_id) { // remove the vertex, as well as all the incident edges

	vertex_weights.erase(u_id); // step 1: remove from all_vertices
	adj_list.erase(u_id); // step 2: remove from adj_list
	for (auto& x: adj_list){ // x == pair<int, unordered_map<int,T>>
		x.second.erase(u_id); // x.second == unordered_map<int, T>
	}

}

/*
	part 2: add, check, remove edge
*/

template <typename T>
void directed_graph<T>::add_edge(const int& u_id, const int& v_id, const T& uv_weight) {

	if(contains(u_id) && contains(v_id)){ // add the edge only if both vertices are in the graph and the edge is not in the graph
		if(adj_list[u_id].find(v_id)==adj_list[u_id].end()){
			adj_list[u_id].insert({v_id, uv_weight});
		}
	}

}

template <typename T>
size_t directed_graph<T>::num_edges() const {

	size_t count = 0;
	for (auto& x: adj_list){ // x == pair<int, unordered_map<int,T>>
		count += x.second.size(); // x.second == unordered_map<int, T>
	}
	return count;

}

template <typename T>
vector<pair<vertex<T>,vertex<T>>> directed_graph<T>::get_edges() {

	vector<pair<vertex<T>,vertex<T>>> v;

	for (auto& x : adj_list){ // x == pair<int, unordered_map<int,T>>
		for (auto& y : adj_list[x.first]){ // x == pair<int, unordered_map<int,T>>
			 v.push_back(pair(vertex<T>(x.first, vertex_weights[x.first]), vertex<T>(y.first, vertex_weights[y.first])));
		}
	}

	return v;

}

template <typename T>
bool directed_graph<T>::adjacent(const int& u_id, const int& v_id) {

	if(contains(u_id) && contains(v_id)){ // first make sure the two vertices are in the graph
		if(adj_list[u_id].find(v_id)!=adj_list[u_id].end()){
			return true;
		}
	}
	return false;

}

template <typename T>
vector<vertex<T>> directed_graph<T>::get_neighbours(const int& u_id) {

	vector<vertex<T>> v;

	if(contains(u_id)){ // first make sure the vertex is in the graph
		for (auto x: adj_list[u_id]){ // adj_list[u_id] is an unordered_map<int, T>
			v.push_back(vertex<T>(x.first, vertex_weights[x.first]));
		}
	}

	return v;
}

template <typename T>
vector<vertex<T>> directed_graph<T>::get_second_order_neighbours(const int& u_id) {

	vector<vertex<T>> v;

	if(contains(u_id)){ // first make sure the vertex is in the graph
		for (auto x: adj_list[u_id]){ // adj_list[u_id] is an unordered_map<int, T>
			for (auto y: adj_list[x.first]){
				if(!find_id_in_vector(y.first, v) && y.first!=u_id){
					v.push_back(vertex<T>(y.first, vertex_weights[y.first]));
				}
			}
		}
	}

	return v;
}

template <typename T>
size_t directed_graph<T>::in_degree(const int& u_id) {

	size_t count = 0;
	if(contains(u_id)){ // first make sure the vertex is in the graph
		for (auto& x: adj_list){ // x == pair<int, unordered_map<int,T>>
			if(x.second.find(u_id)!=x.second.end()){ // x.second == unordered_map<int, T>
				count++;
			}
		}
	}
	return count;

}

template <typename T>
size_t directed_graph<T>::out_degree(const int& u_id) {

	if(contains(u_id)){ // first make sure the vertex is in the graph
		if(adj_list.find(u_id)!=adj_list.end()){
			return adj_list[u_id].size();
		}
	}
	return 0;

}

template <typename T>
size_t directed_graph<T>::degree(const int& u_id) {

	return in_degree(u_id) + out_degree(u_id);

}

template <typename T>
void directed_graph<T>::remove_edge(const int& u_id, const int& v_id) {

	if(contains(u_id) && contains(v_id)){ // first make sure the two vertices are in the graph
		adj_list[u_id].erase(v_id);
	}

}

/*
	graph operations
*/

template <typename T>
vector<vertex<T>> directed_graph<T>::depth_first(const int& u_id) {

	if(contains(u_id)){

		vector<vertex<T>> v; // the visiting order of vertices, which is the result to return
		stack<int> s; // the vertices yet to visit
		s.push(u_id);

		while(true){ // this is to handle disconnected componets of graphs

			while(!s.empty()){
				int temp = s.top();
				s.pop();
				if(!find_id_in_vector(temp, v)){ // x was not visited
					v.push_back(vertex<T>(temp, vertex_weights[temp]));
					for(vertex<T> x : get_neighbours(temp)){
						s.push(x.id);
					}
				}
			}

			if(v.size()==vertex_weights.size()){
				break;
			}

			for(auto x : vertex_weights){
				if (!find_id_in_vector(x.first, v)){
					s.push(x.first);
				}
			}
		}
		return v;

	}
	return vector<vertex<T>>();
}

template <typename T>
vector<vertex<T>> directed_graph<T>::breadth_first(const int& u_id) {

	if(contains(u_id)){

		vector<vertex<T>> v; // the visiting order of vertices, which is the result to return
		queue<int> q; // the vertices yet to visit
		q.push(u_id);

		while(true){ // this is to handle disconnected componets of graphs

			while(!q.empty()){
				int temp = q.front();
				q.pop();
				if(!find_id_in_vector(temp, v)){ // x was not visited
					v.push_back(vertex<T>(temp, vertex_weights[temp]));
					for(vertex<T> x : get_neighbours(temp)){
						q.push(x.id);
					}
				}
			}

			if(v.size()==vertex_weights.size()){
				break;
			}

			for(auto x : vertex_weights){
				if (!find_id_in_vector(x.first, v)){
					q.push(x.first);
				}
			} 
		} // end while
		return v;
	}// end while
	return vector<vertex<T>>();
}

template <typename T>
bool directed_graph<T>::reachable(const int& u_id, const int& v_id) {

	if(contains(u_id) && contains(v_id)){ // first make sure the two vertices are in the graph
		
		if(u_id==v_id){
			return true;
		}

		set<int> visited; // the vertices that were visited
		queue<int> q; // the vertices yet to visit
		q.push(u_id);

		while(!q.empty()){
			int temp = q.front();
			q.pop();
			if(visited.find(temp) == visited.end()){ // x was not visited
				if(temp==v_id){
					return true;
				}
				visited.insert(temp);
				for(vertex<T> x : get_neighbours(temp)){
					q.push(x.id);
				}
			}
		} // end while
	}
	return false;
}

template <typename T>
bool directed_graph<T>::contain_cycles() {

	for(auto k : vertex_weights){ // check cycle for every vertex

		set<int> visited; // the vertices that were visited
		queue<int> q; // the vertices yet to visit
		q.push(k.first);

		bool is_first_iteration = true;
		while(!q.empty()){

			int temp = q.front();
			q.pop();

			if (temp==k.first & ! is_first_iteration){ // x was visited
				// cout << "contain cycle on node: " << temp << endl;
				return true;
			}
			if(visited.find(temp) == visited.end()){ // x was not visited
				visited.insert(temp);
				for(vertex<T> x : get_neighbours(temp)){
					q.push(x.id);
				}
			}
			is_first_iteration = false;
		}
	}
	return false;
}

template <typename T>
directed_graph<T> directed_graph<T>::out_tree(const int& u_id) {

	directed_graph<T> o_tree;

	o_tree.add_vertex(vertex<T>(u_id, this->vertex_weights[u_id]));

	while(true){

		T min_weight = numeric_limits<double>::max();
		int min_start_node = -1;
		int min_end_node = -1;

		for(auto v : o_tree.vertex_weights){ // every node in o_tree
			for(auto e : this->adj_list[v.first]){ // every end_node of a node in graph
				if(o_tree.vertex_weights.find(e.first)==o_tree.vertex_weights.end() && e.second < min_weight){
					min_weight = e.second;
					min_start_node = v.first;
					min_end_node = e.first;
				}
			}
		}

		if(min_start_node < 0 || min_end_node < 0){
			break;
		}else{
			o_tree.add_vertex(vertex<T>(min_end_node, this->vertex_weights[min_end_node]));
			o_tree.add_edge(min_start_node, min_end_node, min_weight);
		}

	}

	return o_tree;
}

template <typename T>
vector<vertex<T>> directed_graph<T>::pre_order_traversal(const int& u_id, directed_graph<T>& o_tree) {

	vector<vertex<T>> v;

	v.push_back(vertex(u_id, o_tree.vertex_weights[u_id])); // visit the root
	for(auto e : o_tree.adj_list[u_id]){ // traverse subtrees
		for(auto x : pre_order_traversal(e.first, o_tree)){
			v.push_back(x);
		}
	}

	return v;
}

template <typename T>
vector<vertex<T>> directed_graph<T>::in_order_traversal(const int& u_id, directed_graph<T>& o_tree) {

	vector<vertex<T>> v;

	if(o_tree.adj_list[u_id].size() == 0){

		v.push_back(vertex(u_id, o_tree.vertex_weights[u_id])); // visit the root

	}else{

		bool visited_first_child = false;
		for(auto e : o_tree.adj_list[u_id]){ // traverse subtrees
			for(auto x : in_order_traversal(e.first, o_tree)){
				// cout << e.first << ", " << x.id << endl;
				v.push_back(x);
				if(!visited_first_child){
					v.push_back(vertex(u_id, o_tree.vertex_weights[u_id])); // visit the root
					visited_first_child = true;
				}
			}
		}

	}

	return v;
}

template <typename T>
vector<vertex<T>> directed_graph<T>::post_order_traversal(const int& u_id, directed_graph<T>& o_tree) {
	
	vector<vertex<T>> v;

	for(auto e : o_tree.adj_list[u_id]){ // traverse subtrees
		for(auto x : post_order_traversal(e.first, o_tree)){
			v.push_back(x);
		}
	}
	v.push_back(vertex(u_id, o_tree.vertex_weights[u_id])); // visit the root

	return v;
}

template <typename T>
vector<vertex<T>> directed_graph<T>::significance_sorting() {

	vector<vertex<T>> v;

	vector<pair<int, T>> vertex_weights_vector(vertex_weights.begin(), vertex_weights.end());
	sort(vertex_weights_vector.begin(), vertex_weights_vector.end(), comp);

	for(auto x : vertex_weights_vector){
		v.push_back(vertex<T>(x.first, x.second));
	}

	return v;

}

#endif

