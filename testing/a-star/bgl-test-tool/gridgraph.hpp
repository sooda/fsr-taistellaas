#ifndef GRIDGRAPH_HPP
#define GRIDGRAPH_HPP

#include "gridutil.hpp"
#include <boost/graph/adjacency_iterator.hpp>
#include <boost/iterator/iterator_facade.hpp>

/*
 * for debug prints 
 */
std::ostream& operator<<(std::ostream& os, const gridedge e) {
	os << e.first << "-" << e.second;
}

/*
 * iterate all edges that leave from a given vertex
 */
class grid_out_edge_iterator
	: public boost::iterator_facade<
	  grid_out_edge_iterator,
	  gridedge,
	  std::forward_iterator_tag> {
public:
	grid_out_edge_iterator() : graph(NULL), dir(-1), vertex(), edge(vertex, vertex) {}
	grid_out_edge_iterator(const Grid* g, gridvertex vert = gridvertex())
			: graph(g), dir(42), vertex(vert), edge(vert, vert) {
		if (vert.x == -1 && vert.y == -1) {
			dir = -1;
		} else {
			increment();
		}
	}
private:
	friend class boost::iterator_core_access;
	gridedge& dereference() const {
		return edge;
	}
	bool equal(const grid_out_edge_iterator& other) const {
		if (graph == NULL || other.graph == NULL)
			throw std::runtime_error("Bad things");
		bool b =
			graph == other.graph
			&& ((dir == other.dir && dir == -1) || edge == other.edge);
		return b;
	}
	void increment() {
		if (dir == -1) {
			throw std::runtime_error("incrementing singular out edge iterator");
		}
		if (dir == 42) {
			dir = -1;
		}
		gridvertex dest;
		while (++dir != 8) {
			static const int deltas[] = {
				 1,  0,
				 1,  1,
				 0,  1,
				-1,  1,
				-1,  0,
				-1, -1,
				 0, -1,
				 1, -1
			};
			int dx = deltas[2*dir];
			int dy = deltas[2*dir + 1];
			dest = gridvertex(vertex.x + dx, vertex.y + dy, graph);
			if (graph->free_at(dest.x, dest.y))
				break;
		}
		if (dir == 8) {
			dir = -1;
			dest = vertex;
		}
		edge = gridedge(vertex, dest);
	}
	const Grid* graph;
	int dir;
	gridvertex vertex;
	mutable gridedge edge;
};


/*
 * graph class for boost: can be searched
 */
class gridgraph {
	public:
		gridgraph(Grid& data) : data(data) {}
		typedef gridvertex vertex_descriptor;
		typedef gridedge edge_descriptor;
		typedef boost::undirected_tag directed_category;
		typedef boost::disallow_parallel_edge_tag edge_parallel_category;
		typedef boost::incidence_graph_tag traversal_category;
		typedef grid_out_edge_iterator out_edge_iterator;
		typedef size_t degree_size_type;
		typedef size_t edges_size_type;
		typedef size_t vertices_size_type;

		// the following ones not needed
		typedef void in_edge_iterator;
		typedef void vertex_iterator;
		typedef void edge_iterator;
		typedef void adjacency_iterator;
		//typedef typename boost::adjacency_iterator_generator<gridgraph<Grid>, vertex_descriptor, out_edge_iterator>::type adjacency_iterator; // FIXME: needed?
		typedef void edge_property_type;
		typedef void vertex_property_type;
		const Grid& grid() const { return data; }
	private:
		Grid& data;
};


/*
 * Just a generic class for calculating the heuristic in a graph
 */
template <class Graph, class Cost>
class grid_heuristic { // : public astar_heuristic<Graph, Cost>
	public:
		typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;

		grid_heuristic(Vertex goal) : goal(goal) {}
		Cost operator()(Vertex u) {
			Cost dx = abs(goal.x - u.x);
			Cost dy = abs(goal.y - u.y);
			// first diagonally, then the remaining steps in a straight line
			int diagonal_steps = std::min(dx, dy);
			int straight_steps = std::max(dx, dy) - diagonal_steps;
			return diagonal_steps * sqrt(2) + straight_steps;
		}

	private:
		Vertex goal;
};

/*
 * This exception stops the search that would otherwise go to all nodes
 */
class found_goal {};

/*
 * Visitor for boost. Check for goal when examining vertices
 */
template <class Vertex>
class grid_visitor : public boost::default_astar_visitor {
	public:
		grid_visitor(Vertex goal) : goal(goal) {}
		template <class Graph>
		void examine_vertex(Vertex u, const Graph& g) {
			// TODO: keep track of vertices visited; if too much, report "not reachable"
			// cout << "Examining " << u << " (goal " << goal << ")" << endl;
			if (u == goal) {
				throw found_goal();
			}
		}
		// what was this?
		//template <class Graph>
		//void discover_vertex(Vertex u, const Graph& g) {
		//}
	private:
		Vertex goal;
};


// TODO: do we need these typedefs, or is it fine to just use our own types straight away?
typedef boost::graph_traits<gridgraph>::out_edge_iterator out_edge_iterator;
typedef boost::graph_traits<gridgraph>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<gridgraph>::edge_descriptor edge_descriptor;
typedef boost::graph_traits<gridgraph>::degree_size_type degree_size_type;
typedef float cost;

// incidencegraph
std::pair<out_edge_iterator, out_edge_iterator> out_edges(vertex_descriptor u, const gridgraph& g) {
	return std::make_pair(out_edge_iterator(&g.grid(), u), out_edge_iterator(&g.grid()));
}
degree_size_type out_degree(vertex_descriptor u, const gridgraph&) {
	throw std::runtime_error("shouldn't call this");
	return 0;
}
vertex_descriptor source(edge_descriptor e, const gridgraph&) {
	return e.first;
}
vertex_descriptor target(edge_descriptor e, const gridgraph&) {
	return e.second;
}
size_t num_vertices(const gridgraph& g) {
	return g.grid().width() * g.grid().height();
}

// TODO: traits?
class edge_weight_map;
namespace boost {
	template <>
	struct property_map<gridgraph, boost::edge_weight_t> {
		typedef edge_weight_map type;
		typedef const edge_weight_map const_type;
	};
}

/*
 * Edge weight is different for diagonal edges and axis-aligned edges
 */
struct edge_weight_map {
	typedef float value_type;
	typedef float reference;
	typedef edge_descriptor key_type;
	typedef boost::readable_property_map_tag category;

	reference operator[](key_type e) const {
		if (e.first.x == e.second.x || e.first.y == e.second.y)
			return 1.0;
		return 1.41;
	}
};

// TODO: can't we default these? boost::reference_wrapper etc?
typedef boost::property_map<gridgraph, boost::edge_weight_t>::const_type const_edge_weight_map;
typedef boost::property_traits<const_edge_weight_map>::reference edge_weight_map_value_type;
typedef boost::property_traits<const_edge_weight_map>::key_type edge_weight_map_key;

// PropertyMap
edge_weight_map_value_type get(const_edge_weight_map& pmap, edge_weight_map_key e) {
	edge_weight_map_value_type v = pmap[e];
	return v;
}

// ReadablePropertyGraph
const_edge_weight_map get(boost::edge_weight_t, const gridgraph&) {
	return const_edge_weight_map();
}
// TODO: use this and a big vector instead of std::map
struct grid_vertex_index_map;
namespace boost {
	template <>
	struct property_map<gridgraph, boost::vertex_index_t> {
		typedef grid_vertex_index_map type;
		typedef const type const_type;
	};
}

/*
 * Transform vertex to a numeric id for indexing arrays with vertices
 */
struct grid_vertex_index_map {
	typedef int value_type;
	typedef int reference;
	typedef vertex_descriptor key_type;
	typedef boost::readable_property_map_tag category;

	grid_vertex_index_map(const Grid& grid) : grid(grid) {}

	reference operator[](key_type k) const {
		int v = k.y * grid.width() + k.x;
		return v;
	}
private:
	const Grid& grid;
};

//XXX: ::const_type
typedef boost::property_map<gridgraph, boost::vertex_index_t>::const_type my_vertex_index_map;
typedef boost::property_traits<my_vertex_index_map>::reference vertex_index_map_value_type;
typedef boost::property_traits<my_vertex_index_map>::key_type vertex_index_map_key;


// PropertyMap valid expressions
vertex_index_map_value_type get(const my_vertex_index_map& pmap, vertex_index_map_key e) {
	return pmap[e];
}

// ReadablePropertyGraph valid expressions
// WeightMap weightmap = get(boost::vertex_index, grappi);
my_vertex_index_map get(boost::vertex_index_t, const gridgraph& gg) {
	return my_vertex_index_map(gg.grid());
}

template <class lol>
void put(my_pred_map& m, lol k, lol v) {
	m[k] = v;
}
namespace boost {
	template <>
		struct property_traits<my_pred_map> {
			typedef vertex_descriptor value_type;
			typedef vertex_descriptor& reference;
			typedef vertex_descriptor key_type;
			typedef boost::read_write_property_map_tag category;
		};
}

class my_dist_map : public std::map<vertex_descriptor, cost> {
	public:
	cost & operator[](const vertex_descriptor& v) {
		auto it = this->find(v);
		if (it == this->end()) {
			this->insert(std::make_pair(v, std::numeric_limits<cost>::max()));
		}
		return self::operator[](v);
	}
	private:
	typedef map<vertex_descriptor, cost> self;
};

void put(my_dist_map& m, gridvertex k, float v) {
	m[k] = v;
}
namespace boost {
	template <>
		struct property_traits<my_dist_map> {
			typedef cost value_type;
			typedef cost& reference;
			typedef vertex_descriptor key_type;
			typedef boost::read_write_property_map_tag category;
		};
}
cost get(my_dist_map& m, gridvertex k) {
	return m[k];
}
namespace boost {
	template <>
	struct property_traits<boost::reference_wrapper<my_dist_map>> {
		typedef cost value_type;
	};
}
#endif
