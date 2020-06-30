#ifndef DIRECTED_GRAPH_H
#define DIRECTED_GRAPH_H

#include<iostream>
#include<vector>
#include<unordered_map>
#include<unordered_set>
#include<set>
#include<queue>
#include<stack>
#include<algorithm>

using namespace std;

template <typename T> class vertex {
public:
	int id; // unique identifer for each vertex
	T weight; // int, double, char, string, ...
	vertex(int v_id, T v_weight) : id(v_id), weight(v_weight) {};//create pair
};

template <typename T> class directed_graph {

private:
	unordered_map <int, T> vertices; 
	/*stores all vertices in the graph, as well as the weights
	each element is a pair(id, weight) for a vertex <key, value>
	because verticies are unique, hence we can utilize a map
	*/
	unordered_map<int, unordered_map<int, T> > adj_list; 
	/*adj_list stores all edges in the graph, as well as the edges' weights.
	each element is a pair(vertex, the neighbours of this vertex)
	each neighbour is also a pair 
	(neighbour_vertex, weight for edge from vertex to neighbour_vertex)
	*/
public:
	directed_graph(); //A constructor for directed_graph. The graph should start empty.
	~directed_graph(); //A destructor. Depending on how you do things, this may not be necessary.
	bool contains(int source_vertex);
	bool adjacent(int source_vertex, int destination_vertex) ; //Returns true if the first vertex is adjacent to the second, false otherwise
	void add_vertex(vertex<T> u); //Adds the passed in vertex to the graph (with no edges).
	void add_edge(int source_vertex, int destination_vertex, T edge_weight);//Adds a weighted edge from the first vertex to the second.
	void remove_vertex(int source_vertex); //Removes the given vertex. Should also clear any incident edges.
	void remove_edge(int source_vertex,  int); //Removes the edge between the two vertices, if it exists.
	
	size_t in_degree(int source_vertex) ; //Returns number of edges coming in to a vertex.
	size_t out_degree(int source_vertex) ; //Returns the number of edges leaving a vertex.
	size_t degree(int source_vertex) ; //Returns the degree of the vertex (both in edges and out edges).
	
	size_t num_vertices() ; //Returns the total number of vertices in the graph.
	size_t num_edges() ; //Returns the total number of edges in the graph.
	
	vector<vertex<T> > get_vertices(); //Returns a vector containing all the vertices.
	vector<vertex<T> > get_neighbours(int source_vertex); //Returns a vector containing all the vertices reachable from the given vertex. The vertex is not considered a neighbour of itself.
	vector<vertex<T> > get_second_order_neighbours(int source_vertex); // Returns a vector containing all the second_order_neighbours (i.e., neighbours of neighbours) of the given vertex. A vector cannot be considered a second_order_neighbour of itself.
	
	bool reachable(int source_vertex, int destination_vertex) ; //Returns true if the second vertex is reachable from the first (can you follow a path of out-edges to get from the first to the second?). Returns false otherwise.
	bool contain_cycles() ; // Return true if the graph contains cycles (there is a path from any vertices directly/indirectly to itself), false otherwise.

	vector<vertex<T> > depth_first(int source_vertex); //Returns the vertices of the graph in the order they are visited in by a depth-first traversal starting at the given vertex.
	vector<vertex<T> > breadth_first(int source_vertex); //Returns the vertices of the graph in the order they are visisted in by a breadth-first traversal starting at the given vertex.
	
	directed_graph<T> out_tree(int source_vertex); //Returns a tree starting at the given vertex using the out-edges. This means every vertex in the tree is reachable from the root.
	vector<vertex<T> > pre_order_traversal(int source_vertex, directed_graph<T> binary_tree); // returns the vertices in the visiting order of a pre-order traversal of the tree starting at the given vertex.
	vector<vertex<T> > in_order_traversal(int source_vertex, directed_graph<T> binary_tree); // returns the vertices in the visiting order of an in-order traversal of the tree starting at the given vertex.
	vector<vertex<T> > post_order_traversal(int source_vertex, directed_graph<T> binary_tree); // returns the vertices in ther visitig order of a post-order traversal of the tree starting at the given vertex.
	vector<vertex<T> > significance_sorting(); // Return a vector containing a sorted list of the vertices in descending order of their significance.
};

template <typename T> directed_graph<T>::directed_graph() {}
template <typename T> directed_graph<T>::~directed_graph() {}
template <typename T> bool directed_graph<T> ::contains(int source_vertex){
	if(vertices.find(source_vertex) != vertices.end()){ 
		return true;
	}
	return false;
}
template <typename T> bool directed_graph<T>::adjacent(int source_vertex, int destination_vertex)  { 
	if (contains(source_vertex) && contains(destination_vertex)) { 
		if(adj_list[source_vertex].find(destination_vertex)!=adj_list[source_vertex].end()){
			return true;
		}
	}
	return false;
	
}
template <typename T> void directed_graph<T>::add_vertex(vertex<T> u) {
	if(!contains(u.id)){
		vertices.insert({u.id, u.weight}); 
		adj_list[u.id] = unordered_map<int, T>(); 
	}
}
template <typename T> void directed_graph<T>::add_edge(int source_vertex, int destination_vertex, T edge_weight) {
	if(contains(source_vertex) && contains(destination_vertex)){
		if(adj_list[source_vertex].find(destination_vertex) ==adj_list[source_vertex].end()){ 
			adj_list[source_vertex].insert({destination_vertex, edge_weight});  
		}
	}
}
template <typename T> void directed_graph<T>::remove_edge(int source_vertex, int destination_vertex) {
   if (contains(source_vertex) && contains(destination_vertex)){
		if(adj_list[source_vertex].find(destination_vertex)!=adj_list[source_vertex].end()){
			adj_list[source_vertex].erase(destination_vertex);
		}
	}
}
template <typename T> void directed_graph<T>::remove_vertex(int source_vertex) { 
	vertices.erase(source_vertex); 
	adj_list.erase(source_vertex); 
	for (auto& x: adj_list){ 
		x.second.erase(source_vertex);
		}
}

template <typename T> size_t directed_graph<T>::in_degree(int source_vertex) { 
	size_t in_degree = 0;
	if(contains(source_vertex)){
		for(int i = 0; i < adj_list.size(); i++){
			if(adj_list[i].find(source_vertex)!=adj_list[i].end()){
				++in_degree;
			}
		}
	}
	return in_degree;
}
template <typename T> size_t directed_graph<T>::out_degree(int source_vertex)  { 
	return contains (source_vertex) ? adj_list[source_vertex].size() : 0;
}
template <typename T> size_t directed_graph<T>::degree(int source_vertex)  { 
	size_t degree = in_degree(source_vertex) + out_degree(source_vertex);
	return degree;
}

template <typename T> size_t directed_graph<T>::num_vertices() { 
	return vertices.size();
}
template <typename T> size_t directed_graph<T>::num_edges() { 
	int number_of_edges = 0;
	for (int i = 0; i< adj_list.size(); i++){
		for( auto x: adj_list[i]){
			++number_of_edges;
		}
	}
	return number_of_edges;
}

template <typename T> vector<vertex<T> > directed_graph<T>::get_vertices() {
	vector<vertex<T>> vertex_list;
	for(auto x: vertices){ 
		vertex_list.push_back(vertex<T>(x.first, x.second)); 
	}
	return vertex_list; 
}
template <typename T> vector<vertex<T> > directed_graph<T>::get_neighbours(int source_vertex) {
vector<vertex<T>> neighbours;
	if(contains(source_vertex)){ 
		for (auto element: adj_list[source_vertex]){ 
			neighbours.push_back(vertex<T>(element.first, vertices[element.first]));
		}
	}
	return neighbours;
}
template <typename T> vector<vertex<T> > directed_graph<T>::get_second_order_neighbours(int source_vertex) { 
	vector<vertex<T> > neighbours = get_neighbours(source_vertex);
	vector<vertex<T> > second_neighbours;
	bool duplicate = false;
	for(auto x : neighbours){
		for(auto y: adj_list[x.id]){
			duplicate = false;
			for(auto z : second_neighbours){
				if(z.id == y.first ){
					duplicate = true;
					break;
				}
			}
			if(y.first == source_vertex){
				duplicate = true;
			}
			if(!duplicate){
				second_neighbours.push_back(vertex<T>(y.first, vertices[y.first]));
			}
		}
	}
	return second_neighbours; 
}

template <typename T> bool directed_graph<T>::reachable(int source_vertex, int destination_vertex) { 
	bool isReachable = false;
	bool visited[num_vertices()];
    for (unsigned i = 0; i<num_vertices(); i++){
        visited[i] = false;
    } 

	queue<T>unvisited_queue; 
	
	if(source_vertex == destination_vertex){
		isReachable = true;
	}
	else{
	unvisited_queue.push(source_vertex); 
    while (!unvisited_queue.empty()){
        T current_vertex = unvisited_queue.front(); 
        unvisited_queue.pop(); 
		if (!visited[current_vertex]){  
            visited[current_vertex] = true;
			for (unsigned i = 0; i< num_vertices(); i++){
               if(adjacent(current_vertex,i)){
					unvisited_queue.push(i);
				}
				if(current_vertex == destination_vertex){
						isReachable = true;
				}
            }
	    }        
	}
	}
	return isReachable;
}
template <typename T> bool directed_graph<T>::contain_cycles() { 
	bool containCycles = false;
	vector<vertex<T> > vertex_list = get_vertices();
	/*If adjacency list has 1 or less nodes, it is a cycle.*/
	if(adj_list.size() <=1){
		containCycles = true;
		return containCycles;
	}
	for(auto i: vertex_list){
		for(auto x: get_neighbours(i.id)){
			if(x.id==i.id){ 
				containCycles = true;
			}	
		}	
	}
	return containCycles;
}

template <typename T> vector<vertex<T> > directed_graph<T>::depth_first(int start_vertex) { 
	vector<vertex<T>> depth_first_order; 
	bool visited[num_vertices()];
    for (unsigned i = 0; i<num_vertices(); i++){
        visited[i] = false;
    } 
	stack<T>unvisited_stack;
	
	unvisited_stack.push(start_vertex);
    while (!unvisited_stack.empty()){
        T current_vertex = unvisited_stack.top();
        unvisited_stack.pop(); 

		if (!visited[current_vertex]){  
            visited[current_vertex] = true;
            
			depth_first_order.push_back(vertex<T>(current_vertex, vertices[current_vertex]));

			for (unsigned i = 0; i< num_vertices(); i++){
               if(adjacent(current_vertex,i)){
					unvisited_stack.push(i);	
				}
            
			}
		}
	           
	}
	return depth_first_order;
}
template <typename T> vector<vertex<T> > directed_graph<T>::breadth_first(int start_vertex) { 
	vector<vertex<T>> breadth_first_order;
	queue<T> unvisited_queue;
	bool visited[num_vertices()];
    for (unsigned i = 0; i<num_vertices(); i++){
        visited[i] = false;
    }

	unvisited_queue.push(start_vertex);

	while (!unvisited_queue.empty()){
        T current_vertex= unvisited_queue.front();
        unvisited_queue.pop();
		
		if (!visited[current_vertex]){
            visited[current_vertex]=true;
            breadth_first_order.push_back(vertex<T>(current_vertex, vertices[current_vertex]));
			for (unsigned i =0; i< num_vertices(); i++){
                if(adjacent(current_vertex,i)){
                    unvisited_queue.push(i);
                }
            }
		}
    }
	return breadth_first_order;
}

template <typename T> directed_graph<T> directed_graph<T>::out_tree(int source_vertex) {
	directed_graph<T> tree;
	/*bool visited[get_vertices().size()];
	for (unsigned i = 0; i < get_vertices().size(); i++){
		visited[i] = false;
	}
	queue<pair<int,int>> unprocessed;
	unprocessed.push({source_vertex, source_vertex});
	
	while (!unprocessed.empty()){
		pair<int, int> n = unprocessed.front();
		unprocessed.pop();
		if (!visited[n.first]){
			visited[n.first] = true;
			tree.add_vertex(vertices[n.first]);
            tree.add_vertex(vertices[n.second]);
            tree.add_edge(n.second, n.first, 0);
			for (unsigned i = 0; i < get_vertices().size(); i++){
				if (adj_list[n.first][i]){
					unprocessed.push({i, n.first});
				}
			}
		}
	}*/
	
	return tree;
}
template <typename T> vector<vertex<T> > directed_graph<T>::pre_order_traversal(int source_vertex, directed_graph<T> binary_tree) { 
	vector<vertex<T>> pre_order_vector;
	stack<T> unvisited_stack;

	bool visited[num_vertices()];
    for (unsigned i = 0; i<num_vertices(); i++){
        visited[i] = false;
    }

	unvisited_stack.push(source_vertex);
	pre_order_vector.push_back(vertex<T>(source_vertex, vertices[source_vertex]));

	for(unsigned i = binary_tree.num_vertices(); i!=0; i--){
        if(adjacent(i, binary_tree.get_vertices().at(i-1).id)){
			if(!visited[i-1]){
				visited[i-1] = true;
				unvisited_stack.push(i-1);
				pre_order_vector.push_back(vertex<T>(i-1, vertices[i-1]));
			}
		}
	}

	for(unsigned i = 0; i< binary_tree.num_vertices(); i++){
		if(adjacent(i, binary_tree.get_vertices().at(i+1).id)){
			if(!visited[i+1]){
				visited[i+1] = true;
				unvisited_stack.push(i+1);
				pre_order_vector.push_back(vertex<T>(i+1, vertices[i+1]));
			}
		}
	}
	return pre_order_vector; 
}
template <typename T> vector<vertex<T> > directed_graph<T>::in_order_traversal(int source_vertex, directed_graph<T> binary_tree) { 
	vector<vertex<T> >in_order_vector;
	stack<T> unvisited_stack;

	bool visited[num_vertices()];
    for (unsigned i = 0; i<num_vertices(); i++){
        visited[i] = false;
    }

	for(unsigned i = binary_tree.num_vertices(); i!=0; i--){
        if(adjacent(i, binary_tree.get_vertices().at(i-1).id)){
			if(!visited[i-1]){
				visited[i-1] = true;
				unvisited_stack.push(i-1);
				in_order_vector.push_back(vertex<T>(i-1, vertices[i-1]));
			}
		}
	}

	unvisited_stack.push(source_vertex);
	in_order_vector.push_back(vertex<T>(source_vertex, vertices[source_vertex]));

	for(unsigned i = 0; i< binary_tree.num_vertices(); i++){
		if(adjacent(i, binary_tree.get_vertices().at(i+1).id)){
			if(!visited[i+1]){
				visited[i+1] = true;
				unvisited_stack.push(i+1);
				in_order_vector.push_back(vertex<T>(i+1, vertices[i+1]));
			}
		}
	}
	return in_order_vector; 
}
template <typename T> vector<vertex<T> > directed_graph<T>::post_order_traversal(int source_vertex, directed_graph<T> binary_tree) { 
	vector<vertex<T> >post_order_vector;
	stack<T> unvisited_stack;

	bool visited[num_vertices()];
    for (unsigned i = 0; i<num_vertices(); i++){
        visited[i] = false;
    }

	for(unsigned i = binary_tree.num_vertices(); i!=0; i--){
        if(adjacent(i, binary_tree.get_vertices().at(i-1).id)){
			if(!visited[i-1]){
				visited[i-1] = true;
				unvisited_stack.push(i-1);
				post_order_vector.push_back(vertex<T>(i-1, vertices[i-1]));
			}
		}
	}
	for(unsigned i = 0; i< binary_tree.num_vertices(); i++){
		if(adjacent(i, binary_tree.get_vertices().at(i+1).id)){
			if(!visited[i+1]){
				visited[i+1] = true;
				unvisited_stack.push(i+1);
				post_order_vector.push_back(vertex<T>(i+1, vertices[i+1]));
			}
		}
	}
		unvisited_stack.push(source_vertex);
	post_order_vector.push_back(vertex<T>(source_vertex, vertices[source_vertex]));
	return post_order_vector; 
}
template <typename T> vector<vertex<T> > directed_graph<T>::significance_sorting() { 
	vector<vertex<T> > sorted_vector;
	vector<vertex<T> > unsorted_vector = get_vertices();
	size_t n = vertices.size();
	//sorted_vector = sort(unsorted_vector, unsorted_vector+n);
	return sorted_vector;
}
#endif