#include "gridsearch.hpp"
#include "gridgraph.hpp"

std::list<vertex_descriptor> gridsearch(const VectorGrid& grid, vertex_descriptor start, vertex_descriptor goal) {
	using namespace std;
	gridgraph g(grid);
	grid_heuristic<gridgraph, cost> heuristic(goal);
	grid_visitor<gridvertex> visitor(goal);
	my_pred_map p; // vec(num_vertices(g)); (vector is faster if indexed with my index map, but it uses more memory)
	my_dist_map d; // vec(num_vertices(g));
	d[start] = 0; // mandatory for some magic reason
	std::map<vertex_descriptor, boost::default_color_type> colors;
	try {
		boost::astar_search_no_init(g, start, heuristic,
				boost::visitor(visitor)
				.predecessor_map(boost::ref(p))
				.distance_map(boost::ref(d))
				.color_map(boost::associative_property_map<map<vertex_descriptor, boost::default_color_type>>(colors))
		);
	} catch (found_goal&) {
		cout << "HURRAA" << endl;
	}
	/*for (auto it = p.begin(); it != p.end(); ++it) {
		cout << it->first << " pred is " << it->second << endl;
	}
	*/
	/*
	for (auto it = d.begin(); it != d.end(); ++it) {
		cout << it->first << " - " << it->second << endl;
	}
	*/
	list<vertex_descriptor> shortest_path;
	if (p[goal] == vertex_descriptor()) {
		cout << "NOT FOUND" << endl;
	} else {
		for(vertex_descriptor v = goal;; v = p[v]) {
			shortest_path.push_front(v);
			if(v == start)
				break;
		}
	}
	cout << "Shortest path:";
	list<vertex_descriptor>::iterator spi = shortest_path.begin();
	cout << *spi;
	for(++spi; spi != shortest_path.end(); ++spi)
		cout << " -> " << *spi;
	cout << endl << "Total travel time: " << d[goal] << endl;
	cout << "search finished" << endl;
	//return make_pair(shortest_path, p);
	return shortest_path;
}
