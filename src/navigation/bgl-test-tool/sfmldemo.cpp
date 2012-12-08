#include <SFML/Graphics.hpp>
#include <SFML/Window/Input.hpp>
#include <iostream>
#include <vector>
#include <list>
#include <cmath>
#include <iterator>
#include <utility> // need for pair?
#include <stdexcept>
#include "gridsearch.hpp"
#include "getturns.hpp"

const int ww = 1024, wh = 768; // win wid, hei
const int sz = 32;
const int gw = ww/sz, gh = wh/sz; // grid size

using namespace std;

void box(sf::RenderWindow& app, int x, int y, const sf::Color& color) {
	app.Draw(sf::Shape::Rectangle(x * sz, y * sz, (x + 1) * sz, (y + 1) * sz, color));
}

int main()
{
	using std::vector;
	list<gridvertex> l;
	vector<pair<vertex, vertex>> corners;
	my_pred_map p;
	sf::RenderWindow App(sf::VideoMode(ww, wh), "mouse buttons for walls, space for routing");
	vector<vector<bool>> grid(gh, vector<bool>(gw));
	VectorGrid container(grid);

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
		bool pressedr = inp.IsMouseButtonDown(sf::Mouse::Right);
		if (pressedr) {
			grid[mgy][mgx] = false;
		}

		if (!(lastx == mgx && lasty == mgy) && !grid[mgy][mgx] && inp.IsKeyDown(sf::Key::Space)) {
			boost::tie(l, p) = gridsearch(grid, gridvertex(mgx, mgy, &container));
			corners = get_turns(l.begin(), l.end());
			lastx = mgx;
			lasty = mgy;
		}
		for (auto it = p.begin(); it != p.end(); ++it) {
			const gridvertex& v = it->second;
			box(App, v.x, v.y, sf::Color::Yellow);
		}
		for (auto it = l.begin(); it != l.end(); ++it) {
			const gridvertex& v = *it;
			box(App, v.x, v.y, sf::Color::Blue);
		}
		for (auto it = corners.begin(); it != corners.end(); ++it) {
			vertex& v = it->first;
			vertex& d = it->second;
			box(App, v.x, v.y, sf::Color::Magenta);
			App.Draw(sf::Shape::Line((v.x+0.5)*sz, (v.y+0.5)*sz, (v.x+d.x+0.5)*sz, (v.y+d.y+0.5)*sz, 2, sf::Color::Cyan));
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
					box(App, x, y, sf::Color::Red);
				}
			}
		}

		App.Display();
	}

	return EXIT_SUCCESS;
}
