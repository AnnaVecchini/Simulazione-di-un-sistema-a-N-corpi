#include <SFML/Graphics.hpp>
#include <array>
#include <exception>
#include <iostream>

#include "n_bodies.hpp"

sf::Vector2f adapt_to_screen(pf::Vec2D const &v, sf::RenderWindow const &window,
                             float scale) {
  sf::Vector2u size = window.getSize();
  float const center_x = static_cast<float>(size.x) / 2.0f;
  float const center_y = static_cast<float>(size.y) / 2.0f;
  float screen_x = center_x + static_cast<float>(v.x) * scale;
  float screen_y = center_y - static_cast<float>(v.y) * scale;
  return {screen_x, screen_y};
}

void draw_bodies(std::vector<pf::Body> const &bodies, sf::RenderWindow &window,
                 float scale) {
  static const std::array<sf::Color, 6> colors = {
      sf::Color::Red,    sf::Color::Green, sf::Color::Blue,
      sf::Color::Yellow, sf::Color::Cyan,  sf::Color::Magenta};

  float const radius = 8.f;

  for (std::size_t i{}; i < bodies.size(); ++i) {
    sf::CircleShape circle{radius};
    circle.setOrigin(radius, radius);
    circle.setFillColor(colors[i % colors.size()]);

    sf::Vector2f screen_r = adapt_to_screen(bodies[i].get_r(), window, scale);

    circle.setPosition(screen_r.x, screen_r.y);
    window.draw(circle);
  }
}

void handle_events(sf::RenderWindow &window, int &steps_per_frame,
                   float &scale) {
  sf::Event event;
  while (window.pollEvent(event)) {
    if (event.type == sf::Event::Closed) window.close();
    if (event.type == sf::Event::KeyPressed) {
      if (event.key.code == sf::Keyboard::Up) {
        ++steps_per_frame;
      }
      if (event.key.code == sf::Keyboard::Down) {
        if (steps_per_frame > 1) steps_per_frame -= 1;
      }
    }
    if (event.type == sf::Event::MouseWheelScrolled) {
      if (event.mouseWheelScroll.wheel == sf::Mouse::VerticalWheel) {
        float delta = event.mouseWheelScroll.delta;

        if (delta > 0) {
          scale *= 1.1f;
        } else if (delta < 0) {
          scale /= 1.1f;
        }
      }
    }
  }
}

int main() {
  try {
    std::vector<pf::Body> bodies =
        pf::load_bodies_from_file("initial_conditions.txt");

    pf::SystemTotals initial = pf::compute_totals(bodies);
    std::cout << "INITIAL TOTAL ENERGY: E(0) = " << initial.E << '\n'
              << "INITIAL TOTAL MOMENTUM: P(0) = (" << initial.P.x << ", "
              << initial.P.y << ")" << '\n'
              << "INITIAL TOTAL ANGULAR MOMENTUM: L(0) = " << initial.L
              << "\n\n";

    pf::Simulation sim(std::move(bodies));

    // create the window
    sf::RenderWindow window(sf::VideoMode(800, 600), "n_bodies simulation");
    window.setFramerateLimit(60);

    int SPF{10};  // it stands for steps_per_frame

    int step{};

    float scale{200.f};

    while (window.isOpen()) {
      handle_events(window, SPF, scale);

      window.clear();
      draw_bodies(sim.get_bodies(), window, scale);
      window.display();

      for (int i{}; i < SPF; ++i) {
        sim.step();
        ++step;
        pf::SystemTotals current = pf::compute_totals(sim.get_bodies());

        pf::print_if_not_conserved('E', initial.E, current.E, step);
        pf::print_if_not_conserved('P', initial.P, current.P, step);
        pf::print_if_not_conserved('L', initial.L, current.L, step);
      }
    }
  } catch (std::exception const &e) {
    std::cerr << "Standard exception caught: " << e.what() << '\n';
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "Unknown exception caught.\n";
    return EXIT_FAILURE;
  }
}