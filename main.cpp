#include <SFML/Graphics.hpp>
#include <array>
#include <iostream>

#include "n_bodies.hpp"

sf::Vector2f adapt_to_screen(pf::Vec2D const &v) {
  float const scale = 200.f;
  float const center_x = 400.f;
  float const center_y = 300.f;
  float screen_x = center_x + static_cast<float>(v.x) * scale;
  float screen_y = center_y - static_cast<float>(v.y) * scale;
  return {screen_x, screen_y};
}

void draw_bodies(std::vector<pf::Body> const &bodies,
                 sf::RenderWindow &window) {
  static const std::array<sf::Color, 6> colors = {
      sf::Color::Red,    sf::Color::Green, sf::Color::Blue,
      sf::Color::Yellow, sf::Color::Cyan,  sf::Color::Magenta};

  for (std::size_t i{}; i < bodies.size(); ++i) {
    auto r = bodies[i].get_r();
    sf::CircleShape circle{8.f};
    circle.setOrigin(8.f, 8.f);
    circle.setFillColor(colors[i % colors.size()]);

    sf::Vector2f screen_r = adapt_to_screen(r);

    circle.setPosition(screen_r.x, screen_r.y);
    window.draw(circle);
  }
}

int main() {
  try {
    std::vector<pf::Body> bodies =
        pf::load_bodies_from_file("initial_conditions.txt");

    pf::SystemTotals initial = pf::compute_totals(bodies);

    pf::Simulation sim(std::move(bodies));

    // create the window
    sf::RenderWindow window(sf::VideoMode(800, 600), "n_bodies simulation");
    window.setFramerateLimit(60);
    int step_per_frame{10};

    int step{};

    while (window.isOpen()) {
      // gestisci gli eventi (movimenti o click del mouse, uso della tastiera,
      // ecc.) avvenuti dal frame precedente
      sf::Event event;
      while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) window.close();
      }

      window.clear();
      draw_bodies(sim.get_bodies(), window);
      window.display();

      for (int i{}; i < step_per_frame; ++i) {
        sim.step();
        ++step;
        pf::SystemTotals current = pf::compute_totals(sim.get_bodies());

        pf::print_total("E", initial.E, current.E, step);
        pf::print_total("P", initial.P, current.P, step);
        pf::print_total("L", initial.L, current.L, step);
      }
    }

  } catch (std::exception const &e) {
    std::cerr << "Caught exception: '" << e.what() << "'\n";
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "Caught unknown exception\n";
    return EXIT_FAILURE;
  }
}