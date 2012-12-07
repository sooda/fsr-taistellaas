#include <SFML/Graphics.hpp>
#include <SFML/Window/Input.hpp>
#include <iostream>
#include <vector>
#include <list>
#include <map>
#include <cmath>
#include <iterator>
#include "astar_search.hpp"
#include <boost/graph/adjacency_iterator.hpp>
#include <utility> // need for pair?
#include <boost/iterator/iterator_facade.hpp>
#include <stdexcept>

const int ww = 1024, wh = 768; // win wid, hei
const int sz = 32;
const int gw = ww/sz, gh = wh/sz; // grid size

using namespace std;

class gridvertex {
	public:
		int x, y; // const
		gridvertex() : x(-1), y(-1) {}
		gridvertex(int x, int y) : x(x), y(y) {}

		friend std::ostream& operator<<(std::ostream& os, const gridvertex& u) {
			os << "(" << u.x << ", " << u.y << ")";
		}
		bool operator==(const gridvertex& other) const {
			return x == other.x && y == other.y;
		}
		bool operator!=(const gridvertex& other) const {
			return !(*this == other);
		}
		// for std::map
		bool operator<(const gridvertex& other) const {
			return id(*this) < id(other);
		}
	private:
		static int id(const gridvertex& v) {
			return v.y * gw + v.x;
		}
};

typedef std::pair<gridvertex, gridvertex> gridedge;

std::ostream& operator<<(std::ostream& os, const gridedge e) {
	os << e.first << "-" << e.second;
}

class Grid {
public:
	virtual bool free_at(const gridvertex& v) const = 0;
};
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
			dest = gridvertex(vertex.x + dx, vertex.y + dy);
			if (graph->free_at(dest))
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


template <class GridData>
class gridgraph : public Grid {
	public:
		gridgraph(GridData& data) : data(data) {}
		bool free_at(const gridvertex& v) const {
			if (v.x < 0 || v.y < 0 || v.x >= gw || v.y >= gh)
				return false;
			if (data[v.y][v.x])
				return false;
			return true;
		}
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

	private:
		GridData& data;
};

template <class Graph, class Cost>
class grid_heuristic { // : public astar_heuristic<Graph, Cost>
	public:
		typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;

		grid_heuristic(Vertex goal) : goal(goal) {}
		Cost operator()(Vertex u) {
			Cost dx = goal.x - u.x;
			Cost dy = goal.y - u.y;
			float f = sqrt(dx * dx + dy * dy); // FIXME, does not work with grid exactly like this?
			return f;
		}

	private:
		Vertex goal;
};

class found_goal {};

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
			//template <class Graph>
			//void discover_vertex(Vertex u, const Graph& g) {
			//}
	private:
		Vertex goal;
};


typedef gridgraph<vector<vector<bool>>> graph;
typedef graph::vertex_descriptor vertex;
typedef float cost;
typedef boost::graph_traits<graph>::out_edge_iterator out_edge_iterator;
typedef boost::graph_traits<graph>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<graph>::edge_descriptor edge_descriptor;
typedef boost::graph_traits<graph>::degree_size_type degree_size_type;


// incidencegraph
std::pair<out_edge_iterator, out_edge_iterator> out_edges(vertex_descriptor u, const graph& g) {
	return make_pair(out_edge_iterator(&g, u), out_edge_iterator(&g));
}
degree_size_type out_degree(vertex_descriptor u, const graph&) {
	throw std::runtime_error("shouldn't call this");
	return 0;
}
vertex_descriptor source(edge_descriptor e, const graph&) {
	return e.first;
}
vertex_descriptor target(edge_descriptor e, const graph&) {
	return e.second;
}
// ??
size_t num_vertices(const graph& g) {
	return gw * gh;
}

// TODO: traits?
class edge_weight_map;
namespace boost {
	template <>
	struct property_map<gridgraph<vector<vector<bool>>>, boost::edge_weight_t> {
		typedef edge_weight_map type;
		typedef const edge_weight_map const_type;
	};
}

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

typedef boost::property_map<graph, boost::edge_weight_t>::const_type const_edge_weight_map;
typedef boost::property_traits<const_edge_weight_map>::reference edge_weight_map_value_type;
typedef boost::property_traits<const_edge_weight_map>::key_type edge_weight_map_key;

// PropertyMap
edge_weight_map_value_type get(const_edge_weight_map& pmap, edge_weight_map_key e) {
	edge_weight_map_value_type v = pmap[e];
	return v;
}

// ReadablePropertyGraph
// WeightMap weightmap = get(boost::edge_weight, grappi);
const_edge_weight_map get(boost::edge_weight_t, const graph&) {
	return const_edge_weight_map();
}
// TODO: use this and a big vector instead of std::map
struct vertexindexasdmap;
namespace boost {
	template <>
	struct property_map<gridgraph<vector<vector<bool>>>, boost::vertex_index_t> {
		typedef vertexindexasdmap type;
		typedef const type const_type;
	};
}

struct vertexindexasdmap {
	typedef int value_type;
	typedef int reference;
	typedef vertex_descriptor key_type;
	typedef boost::readable_property_map_tag category;

	reference operator[](key_type k) const {
		int v = k.y * gw + k.x; // TODO: magic width somewhere
		return v;
	}
};

//XXX: ::const_type
typedef boost::property_map<graph, boost::vertex_index_t>::const_type my_vertex_index_map;
typedef boost::property_traits<my_vertex_index_map>::reference vertex_index_map_value_type;
typedef boost::property_traits<my_vertex_index_map>::key_type vertex_index_map_key;


// PropertyMap valid expressions
vertex_index_map_value_type get(const my_vertex_index_map& pmap, vertex_index_map_key e) {
	return pmap[e];
}

// ReadablePropertyGraph valid expressions
// WeightMap weightmap = get(boost::vertex_index, grappi);
my_vertex_index_map get(boost::vertex_index_t, const graph&) {
	return my_vertex_index_map();
}

class my_pred_map : public map<vertex_descriptor, vertex_descriptor> {};
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

class my_dist_map : public map<vertex_descriptor, cost> {
	public:
	cost & operator[](const vertex_descriptor& v) {
		auto it = this->find(v);
		if (it == this->end()) {
			this->insert(make_pair(v, std::numeric_limits<cost>::max()));
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
	struct property_traits<boost::reference_wrapper<my_dist_map> > {
		typedef cost value_type;
	};
}

pair<list<vertex_descriptor>, my_pred_map> mysearch(vector<vector<bool>>& grid, vertex_descriptor goal) {
	graph g(grid);
	vertex start(0, 0);
	grid_heuristic<graph, cost> heuristic(goal);
	grid_visitor<vertex> visitor(goal);
	my_pred_map p; // vec(num_vertices(g)); (vector is faster if indexed with my index map, but it uses more memory)
	my_dist_map d; // vec(num_vertices(g));
	d[start] = 0; // mandatory for some magic reason
	map<vertex_descriptor, boost::default_color_type> colors;
	try {
		astar_search_no_init(g, start, heuristic,
				boost::visitor(visitor)
				.predecessor_map(boost::ref(p))
				.distance_map(boost::ref(d))
				.color_map(boost::associative_property_map<map<vertex_descriptor, boost::default_color_type>>(colors))
		);
	} catch (found_goal&) {
		cout << "HURRAA" << endl;
	}
	for (auto it = p.begin(); it != p.end(); ++it) {
		cout << it->first << " pred is " << it->second << endl;
	}
	for (auto it = d.begin(); it != d.end(); ++it) {
		cout << it->first << " - " << it->second << endl;
	}
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
	return make_pair(shortest_path, p);
}
int main()
{
	using std::vector;
	list<vertex_descriptor> l;
	my_pred_map p;
	sf::RenderWindow App(sf::VideoMode(ww, wh), "mouse buttons for walls, space for routing");
	vector<vector<bool>> grid(gh, vector<bool>(gw));

	int lastx = 0;
	int lasty = 0;
	while (App.IsOpened())
	{
		// Process events
		sf::Event Event;
		while (App.GetEvent(Event))
		{
			// Close window : exit
			if (Event.Type == sf::Event::Closed)
				App.Close();
		}

		App.Clear();

		const sf::Input& inp = App.GetInput();
		
		int mx = inp.GetMouseX();
		int my = inp.GetMouseY();
		mx &= ~(sz-1);
		my &= ~(sz-1);
		int mgx = mx / sz;
		int mgy = my / sz;
		bool pressed = inp.IsMouseButtonDown(sf::Mouse::Left);
		if (pressed) {
			grid[mgy][mgx] = true;
		}
		bool pressedr= inp.IsMouseButtonDown(sf::Mouse::Right);
		if (pressedr) {
			grid[mgy][mgx] = false;
		}

		if (!(lastx == mgx && lasty == mgy) && !grid[mgy][mgx] && inp.IsKeyDown(sf::Key::Space)) {
			boost::tie(l, p) = mysearch(grid, vertex_descriptor(mgx, mgy));
			lastx = mgx;
			lasty = mgy;
		}
		for (auto it = p.begin(); it != p.end(); ++it) {
			vertex_descriptor& v = it->second;
		App.Draw(sf::Shape::Rectangle(v.x * sz, v.y * sz, (v.x + 1) * sz, (v.y + 1) * sz, sf::Color::Yellow));
		}
		for (auto it = l.begin(); it != l.end(); ++it) {
			vertex_descriptor& v = *it;
		App.Draw(sf::Shape::Rectangle(v.x * sz, v.y * sz, (v.x + 1) * sz, (v.y + 1) * sz, sf::Color::Blue));
		}
		App.Draw(sf::Shape::Rectangle(mx, my, mx+sz-1, my+sz-1, sf::Color::Green));

		for (int i = 0; i < ww; i += sz) {
			App.Draw(sf::Shape::Line(i, 0, i, wh-1, 1, sf::Color::Red));
		}
		for (int i = 0; i < wh; i += sz) {
			App.Draw(sf::Shape::Line(0, i, ww-1, i, 1, sf::Color::Red));
		}
		for (int y = 0; y < gh; y++) {
			for (int x = 0; x < gw; x++) {
				if (grid[y][x]) {
					App.Draw(sf::Shape::Rectangle(x*sz, y*sz, (x+1)*sz, (y+1)*sz, sf::Color::Red));
				}
			}
		}

		App.Display();
	}

	return EXIT_SUCCESS;
}
