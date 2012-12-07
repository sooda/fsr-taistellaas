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

const int ww = 1024, wh = 768; // win wid, hei
const int sz = 32;
const int gw = ww/sz, gh = wh/sz; // grid size

using namespace std;

int main()
{
	using std::vector;
	list<gridvertex> l;
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
			lastx = mgx;
			lasty = mgy;
		}
		for (auto it = p.begin(); it != p.end(); ++it) {
			gridvertex& v = it->second;
		App.Draw(sf::Shape::Rectangle(v.x * sz, v.y * sz, (v.x + 1) * sz, (v.y + 1) * sz, sf::Color::Yellow));
		}
		for (auto it = l.begin(); it != l.end(); ++it) {
			gridvertex& v = *it;
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
