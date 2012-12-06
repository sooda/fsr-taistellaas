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

template <class Grid>
class gridgraph {
	public:
		gridgraph(Grid& grid) : grid(grid) {}

		class vertex_descriptor {
			public:
				int x, y; // const
				vertex_descriptor() : x(-1), y(-1) {}
				vertex_descriptor(int x, int y) : x(x), y(y) {}

				friend std::ostream& operator<<(std::ostream& os, const vertex_descriptor& u) {
					os << "(" << u.x << ", " << u.y << ")";
				}

				bool operator==(const vertex_descriptor& other) const {
					return x == other.x && y == other.y;
				}
				bool operator!=(const vertex_descriptor& other) const {
					return !(*this == other);
				}
				// for std::map
				bool operator<(const vertex_descriptor& other) const {
					return id(*this) < id(other);
				}
			private:
				static int id(const vertex_descriptor& v) {
					return v.y * gw + v.x;
				}
		};
		bool free_at(const vertex_descriptor& v) const {
			if (v.x < 0 || v.y < 0 || v.x >= gw || v.y >= gh)
				return false;
			if (grid[v.y][v.x])
				return false;
			return true;
		}

		class edge_descriptor { // TODO: pair
			public:
				vertex_descriptor u, v; // const
				edge_descriptor(vertex_descriptor u, vertex_descriptor v)
					: u(u), v(v) {}
				edge_descriptor() {}

				bool operator==(const edge_descriptor& other) const {
					return u == other.u && v == other.v;
				}
				bool operator!=(const edge_descriptor& other) const {
					return !(*this == other);
				}
				friend std::ostream& operator<<(std::ostream& os, const edge_descriptor& e) {
					os << e.u << "-" << e.v;
				}
		};

		typedef boost::undirected_tag directed_category;
		typedef boost::disallow_parallel_edge_tag edge_parallel_category;
		typedef boost::incidence_graph_tag traversal_category;

		class out_edge_iterator
			: public boost::iterator_facade<
			  out_edge_iterator,
			  edge_descriptor,
			  std::forward_iterator_tag> {
		public:
			out_edge_iterator() : graph(NULL), dir(-1), vertex(), edge(vertex, vertex) {}
			out_edge_iterator(const gridgraph* g, vertex_descriptor vert = vertex_descriptor())
					: graph(g), dir(42), vertex(vert), edge(vert, vert) {
				cout << "outedge ctor at " << vert << endl;
				if (vert.x == -1 && vert.y == -1) {
					dir = -1;
					cout << "singular" << endl;
				} else {
					cout << "jee" << endl;
					increment();
				}
			}
		private:
			friend class boost::iterator_core_access;
			edge_descriptor& dereference() const {
				cout << "deref " << edge << endl;
				return edge;
			}
			bool equal(const out_edge_iterator& other) const {
				if (graph == NULL || other.graph == NULL)
					throw std::runtime_error("Bad things");
				bool b =
					graph == other.graph
					&& ((dir == other.dir && dir == -1) || edge == other.edge);
				cout << "equal:" << b << " :: " << edge << other.edge << " " << dir << "/" << other.dir << endl;
				return b;
			}
			void increment() {
				cout << "whee incr" << endl;
				cout << "before: " << dir << edge << endl;
				if (dir == -1) {
					throw std::runtime_error("incrementing singular out edge iterator");
				}
				if (dir == 42) {
					dir = -1;
				}
				vertex_descriptor dest;
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
					dest = vertex_descriptor(vertex.x + dx, vertex.y + dy);
					//cout << "try " << dir << ":" << dest << endl;
					if (graph->free_at(dest))
						break;
				}
				if (dir == 8) {
					dir = -1;
					dest = vertex;
				}
				edge = edge_descriptor(vertex, dest);
				cout << "after: " << dir << edge << endl;
			}
			const gridgraph* graph;
			int dir;
			vertex_descriptor vertex;
			mutable edge_descriptor edge;
		};
		typedef size_t degree_size_type;
		typedef size_t edges_size_type;
		typedef size_t vertices_size_type;
		typedef void in_edge_iterator;
		typedef void vertex_iterator;
		typedef void edge_iterator;
		typedef void adjacency_iterator;
		//typedef typename boost::adjacency_iterator_generator<gridgraph<Grid>, vertex_descriptor, out_edge_iterator>::type adjacency_iterator; // FIXME: needed?
		//
		typedef void edge_property_type;
		typedef void vertex_property_type;

	private:
		Grid& grid;
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
			cout << "heuristic " << u << " -- " << goal << ":: " << f << endl;
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
				cout << "Examining " << u << " (goal " << goal << ")" << endl;
				if (u == goal) {
					throw found_goal();
				} else {
					cout << "no: " << u.x << "," << u.y << " -- " << goal.x << "," << goal.y << endl;
				}
			}
			template <class Graph>
			void discover_vertex(Vertex u, const Graph& g) {
				cout << "Discovered " << u << " (goal " << goal << ")" << endl;
			}
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
	cout << "out_edges " << u << endl;
	return make_pair(out_edge_iterator(&g, u), out_edge_iterator(&g)); // TODO
}
degree_size_type out_degree(vertex_descriptor u, const graph&) {
	throw std::runtime_error("shouldn't call this");
	cout << "outdegree" << endl;
	return 0;  // number of out-edges
}
vertex_descriptor source(edge_descriptor e, const graph&) {
	cout << "sourke " << e << "=" << e.u << endl;
	return e.u;
}
vertex_descriptor target(edge_descriptor e, const graph&) {
	cout << "tarket " << e << "=" << e.v << endl;
	return e.v;
}
// ??
size_t num_vertices(const graph& g) {
	cout << "nverts" << endl;
	return gw*gh;
}


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
		cout << "edgeweight map []" << e << endl;
		if (e.u.x == e.v.x || e.u.y == e.v.y)
			return 1.0;
		return 1.41;
	}
};
typedef boost::property_map<graph, boost::edge_weight_t>::const_type const_edge_weight_map;
typedef boost::property_traits<const_edge_weight_map>::reference edge_weight_map_value_type;
typedef boost::property_traits<const_edge_weight_map>::key_type edge_weight_map_key;
// PropertyMap valid expressions
edge_weight_map_value_type get(const_edge_weight_map& pmap, edge_weight_map_key e) {
	cout << "edge map weight " << e << " == " << endl;
	edge_weight_map_value_type v = pmap[e];
	cout << "edge map weight " << e << ": " << v << endl;
	return v;
}

// ReadablePropertyGraph valid expressions
// WeightMap weightmap = get(boost::edge_weight, grappi);
const_edge_weight_map get(boost::edge_weight_t, const graph&) {
	cout << "new edge weight map" << endl;
	return const_edge_weight_map();
}
/*
edge_weight_map_value_type get(boost::edge_weight_t tag, const graph& g, edge_weight_map_key e) {
	cout << "index edge weight omg" << endl;
	return get(tag, g)[e];
}
*/

struct vertexindexasdmap;
namespace boost {
	template <>
	struct property_map<gridgraph<vector<vector<bool>>>, boost::vertex_index_t> {
		typedef vertexindexasdmap type;
		typedef const vertexindexasdmap const_type;
	};
}


struct vertexindexasdmap {
	typedef int value_type;
	typedef int reference;
	typedef vertex_descriptor key_type;
	typedef boost::readable_property_map_tag category;

	reference operator[](key_type k) const {
		int v = k.y * gw + k.x; // TODO: magic width somewhere
		cout << "indexasd map []" << k << ":" << v << endl;
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
	cout << "new vertex index map" << endl;
	return my_vertex_index_map();
}

/*
void put(vertexindexasdmap& map, const vertex_index_map_key& key, vertex_index_map_value_type value) {
	cout << "WRONG" << endl;
	//map[key] = value;
}*/


#if 0

struct vertexcolormaplol;
namespace boost {
	template <>
		struct property_map<gridgraph<vector<vector<bool>>>, boost::vertex_color_t> {
			typedef vertexcolormaplol type;
			typedef vertexcolormaplol const_type;
		};
}


struct vertexcolormaplol {
	typedef int value_type;
	typedef int reference;
	typedef vertex_descriptor key_type;
	typedef boost::read_write_property_map_tag category;

	reference operator[](key_type e) const {
		return 42; // TODO: indices by x,y values, TODO: real reference
	}
};

//XXX: ::const_type
typedef boost::property_map<graph, boost::vertex_color_t>::type my_vertex_color_map_lol;
typedef boost::property_traits<my_vertex_color_map_lol>::reference vertex_color_map_value_type;
typedef boost::property_traits<my_vertex_color_map_lol>::key_type vertex_color_map_key;


// PropertyMap valid expressions
vertex_color_map_value_type get(my_vertex_color_map_lol pmap, vertex_color_map_key e) {
	return pmap[e];
}


// ReadablePropertyGraph valid expressions
// WeightMap weightmap = get(boost::vertex_index, grappi);
my_vertex_color_map_lol get(boost::vertex_index_t, const graph&) {
	return my_vertex_color_map_lol();
}

void put(vertexcolormaplol& map, const vertex_color_map_key& key, vertex_color_map_value_type value) {
	map[key];// = value;
}



typedef boost::property_map<graph, boost::vertex_color_t>::type color_map_t;
#endif


/*
   void put(vertexindexasdmap& map, const vertex_index_map_key& key, boost::default_color_type value) {
   map[key];// = value;
   }
   */

/*
   vertex_index_map_value_type get(boost::vertex_index_t tag, const graph& g, vertex_index_map_key e) {
   return get(tag, g)[e];
   }



*/



// get vertex index map
// mapping K = gridgraph<std::vector<std::vector<bool> > >::vertex_descriptor, V = boost::default_color_type]
// or: no known conversion for argument 1 from ‘gridgraph<std::vector<std::vector<bool> > >::vertex_descriptor’ to ‘const key_type& {aka const long unsigned int&}’
// FIXME TODO XXX: vissiinki verteksi -> size_t -propertyjuttu
// IN: vertex_index_map(VertexIndexMap i_map)
// This maps each vertex to an integer in the range [0, num_vertices(g))
// Note: if you use this default, make sure your graph has an internal vertex_index property.
//boost::identity_property_map get(boost::vertex_index_t, const graph&) {
//const_vertex_weight_map get(boost::vertex_index_t, const graph&) {
//	return const_vertex_index_map();
//}

#if 1
class my_pred_map : public map<vertex_descriptor, vertex_descriptor> {};
#if 1
template <class lol>
void put(my_pred_map& m, lol k, lol v) {
	cout << "LOL PUT LOL" << endl;
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
#endif
#endif
#if 1
class my_dist_map : public map<vertex_descriptor, cost> {
	public:
	cost & operator[](const vertex_descriptor& v) {
		cout << "PENIS " << v << endl;
		auto it = this->find(v);
		if (it == this->end()) {
			cout << "no benis" << endl;
				this->insert(make_pair(v, 9999));
			//this->insert(make_pair(v, std::numeric_limits<cost>::max()));
		}
		return self::operator[](v);
	}
	private:
	typedef map<vertex_descriptor, cost> self;
};

template <class lol, class foo>
void put(my_dist_map& m, lol k, foo v) {
	cout << "ROFL PUT " << &m << k << ":" << v << endl;
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
template <class zing>
cost get(my_dist_map& m, zing k) {
	cout << "ROFL GETDIST " << k << endl;
	cout << m[k] << endl;
	return m[k];
}
#endif
#if 0
namespace boost {
	template <>
		struct property_map<gridgraph<vector<vector<bool>>>, boost::edge_weight_t> {
			typedef edge_weight_map type;
			typedef edge_weight_map const_type;
		};
}
#endif
pair<list<vertex_descriptor>, my_pred_map> mysearch(vector<vector<bool>>& grid, vertex_descriptor goal) {
	graph g(grid);
	vertex start(0, 0);
	grid_heuristic<graph, cost> heuristic(goal);
	grid_visitor<vertex> visitor(goal);
#if 1
	my_pred_map p; //(num_vertices(g));
#endif
#if 1
	my_dist_map d;//(num_vertices(g));
	cout << "ROFL TISSIT " << &d << endl;
	d[start] = 0;
	map<vertex_descriptor, boost::default_color_type> duus;
#endif
	try {
	astar_search_no_init(g, start, heuristic,
			boost::visitor(visitor)
#if 1
			.predecessor_map(boost::ref(p))
#endif
#if 1
#if 0
			.distance_map(d)
#else
			.distance_map(boost::ref(d))
#endif
#endif
			.color_map(boost::associative_property_map<map<vertex_descriptor, boost::default_color_type>>(duus))
			);
	} catch (found_goal&) {
		cout << "HURRAA" << endl;
	}
#if 1
	cout << "loLolOLOL" << endl;
	for (auto it = p.begin(); it != p.end(); ++it) {
		cout << it->first << " pred is " << it->second << endl;
	}
	cout << "ashdisahifhifahfdi" << endl;
	for (auto it = d.begin(); it != d.end(); ++it) {
		cout << it->first << " - " << it->second << endl;
	}
#if 1
	cout << "shbbu" << endl;
	list<vertex_descriptor> shortest_path;
	if (p[goal] == vertex_descriptor()) {
		cout << "NOT FOUND" << endl;
	} else {
		for(vertex_descriptor v = goal;; v = p[v]) {
			cout << "zingzong " << v << endl;
			shortest_path.push_front(v);
			/* if(p[v] == v) // HOX HOX TODO FIXME tolleen se kuuluis koodaa, alku -> alku
				break; */
			if(v == start)
				break;
		}
	}
#endif
#endif
#if 1
	cout << "Shortest path:";
	list<vertex_descriptor>::iterator spi = shortest_path.begin();
	cout << *spi;
	for(++spi; spi != shortest_path.end(); ++spi)
		cout << " -> " << *spi;
#endif
#if 1
	cout << endl << "Total travel time: " << d[goal] << endl;
#endif
	cout << "search finished" << endl;
	return make_pair(shortest_path, p);
}
namespace boost {
	template <>
	struct property_traits<boost::reference_wrapper<my_dist_map> > {
		typedef cost value_type ;
	};
}
int main()
{
	using std::vector;
	vector<vector<bool>> grid0(gh, vector<bool>(gw));
	vector<vector<bool>> grid1(gh, vector<bool>(gw));
	vector<vector<bool>> grid2(gh, vector<bool>(gw));
	list<vertex_descriptor> l;
	my_pred_map p;
	sf::RenderWindow App(sf::VideoMode(ww, wh), "mouse buttons for walls, space for routing");
#if 0
	mysearch(grid0, vertex_descriptor(4, 1));
	mysearch(grid1, vertex_descriptor(5, 2));
	mysearch(grid2, vertex_descriptor(5, 3));
#if 0
	return 0;
#endif
#endif
	vector<vector<bool>> grid(gh, vector<bool>(gw));
	int asdx=1;
	// Create main window

	int lastx = 0;
	int lasty = 0;
	// Start game loop
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
			cout << "ASD ASD ASD " << lastx << " " << lasty << "||" << mgx << " " << mgy << endl;
			//vector<vector<bool>> grid(gh, vector<bool>(gw));

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
