#include <array>
#include <cstddef>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <vector>
#include <deque> //questo
#include <SFML/Graphics.hpp>

#include "n_bodies.hpp"

// Dimensioni della finestra, in pixel
constexpr unsigned int window_width{800};
constexpr unsigned int window_height{600};

// Pixel per unita' di posizione della simulazione (le posizioni nel
// caso Figure-8 sono dell'ordine di 1)
constexpr float scale{200.f};

// Raggio (in pixel) usato per disegnare ogni corpo. Non dipende dalla
// massa: e' un valore fisso, uguale per tutti i corpi, scelto solo
// per una buona resa grafica.
constexpr float radius{8.f};

// Quanti step di simulazione eseguire per ogni fotogramma disegnato:
// con dt molto piccolo (0.001), un solo step per frame renderebbe il
// moto impercettibilmente lento
constexpr int steps_per_frame{20};

constexpr std::size_t trail_length{200}; //questo

// Colori usati per distinguere i corpi, ciclati con l'indice (l'i-esimo
// corpo usa colors[i % colors.size()], cosi' funziona anche con piu'
// corpi che colori disponibili)
std::array<sf::Color, 6> const colors{sf::Color::Red,     sf::Color::Green,
                                       sf::Color::Blue,    sf::Color::Yellow,
                                       sf::Color::Magenta, sf::Color::Cyan};

// Converte una posizione fisica (TDvec) in coordinate pixel,
// centrando l'origine al centro della finestra e capovolgendo l'asse
// y (che in SFML cresce verso il basso, al contrario della convenzione
// fisica usata nella simulazione). Funzione locale a questo file: non
// serve a nessun altro .cpp, quindi non va in n_bodies.hpp.
sf::Vector2f toScreenCoordinates(pf::TDvec const& r) {
  float const x{static_cast<float>(r.x) * scale +
                static_cast<float>(window_width) / 2.f};
  float const y{static_cast<float>(window_height) / 2.f -
                 static_cast<float>(r.y) * scale};
  return {x, y};
}

// Disegna tutti i corpi nella finestra: un cerchio colorato per
// ciascuno, tutti con lo stesso raggio fisso.
void drawBodies(std::vector<pf::Body> const& bodies, sf::RenderWindow& window) {
  for (std::size_t i{0}; i < bodies.size(); ++i) {
    sf::CircleShape shape{radius};
    shape.setFillColor(colors[i % colors.size()]);
    shape.setOrigin(radius, radius);
    shape.setPosition(toScreenCoordinates(bodies[i].r));
    window.draw(shape);
  }
}

void drawTrails(std::vector<std::deque<sf::Vector2f>> const& trails, //questo
                 sf::RenderWindow& window) {
  for (std::size_t i{0}; i < trails.size(); ++i) {
    sf::VertexArray line{sf::LineStrip, trails[i].size()};
    sf::Color const base_color{colors[i % colors.size()]};

    for (std::size_t k{0}; k < trails[i].size(); ++k) {
      float const alpha{static_cast<float>(k) /
                         static_cast<float>(trails[i].size())};
      sf::Color point_color{base_color};
      point_color.a = static_cast<sf::Uint8>(alpha * 255.f);

      line[k].position = trails[i][k];
      line[k].color = point_color;
    }
    window.draw(line);
  }
}

int main() {
  try {
    // Condizioni iniziali lette da file: caso Figure-8 (traccia).
    // Nel file, le masse sono gia' scalate a 1/G. Le condizioni
    // iniziali originali sono pensate per G=1, m=1; con il vero
    // G=6.67e-11 le forze sarebbero troppo deboli per generare la
    // stessa orbita, quindi scelgo m = 1/G in modo che G*m = 1,
    // riproducendo la stessa dinamica del caso originale.
    std::vector<pf::Body> bodies{pf::readBodiesFromFile("initial_conditions.txt")};

    std::vector<std::deque<sf::Vector2f>> trails(bodies.size()); //questo

    // Inizializzo le accelerazioni a(0) PRIMA del ciclo: step() si
    // aspetta che bodies[i].a contenga gia' l'accelerazione corrente
    pf::computeAccelerations(bodies);

    double const E0{pf::computeEnergy(bodies)};
    pf::TDvec const P0{pf::computeMomentum(bodies)};
    double const L0{pf::computeAngularMomentum(bodies)};
    double const tolerance{0.01};  // 1% di tolleranza relativa

    sf::RenderWindow window{sf::VideoMode(window_width, window_height),
                             "N_bodies Simulation"};
    window.setFramerateLimit(60u);

    int frame{0};
    while (window.isOpen()) {
      sf::Event event;
      while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
          window.close();
        }
      }

      for (int i{0}; i < steps_per_frame; ++i) {
        pf::step(bodies);
      }
      for (std::size_t i{0}; i < bodies.size(); ++i) { //questo
       trails[i].push_back(toScreenCoordinates(bodies[i].r));
       if (trails[i].size() > trail_length) {
       trails[i].pop_front();
       }
       }

      // Ogni ~60 frame (circa una volta al secondo, con il limite di
      // 60 fps impostato sopra) stampiamo un riepilogo delle
      // grandezze fisiche conservate, una per riga
      if (frame % 60 == 0) {
        double const K{pf::computeKineticEnergy(bodies)};
        double const U{pf::computePotentialEnergy(bodies)};
        double const E{K + U};
        pf::TDvec const P{pf::computeMomentum(bodies)};
        double const L{pf::computeAngularMomentum(bodies)};

        bool const E_conserved{pf::isEnergyConserved(E0, E, tolerance)};
        bool const P_conserved{pf::isMomentumConserved(bodies, P0, tolerance)};
        bool const L_conserved{
            pf::isAngularMomentumConserved(bodies, L0, tolerance)};

        std::cout << "----- t = " << pf::t << " -----\n";
        std::cout << "Kinetic energy   K = " << K << '\n';
        std::cout << "Potential energy U = " << U << '\n';
        std::cout << "Total energy     E = " << E
                   << "  (is conserved: " << (E_conserved ? "yes" : "no") << ")\n";
        std::cout << "Momentum  Px = " << P.x << '\n';
        std::cout << "Momentum  Py = " << P.y
                   << "  (is conserved: " << (P_conserved ? "yes" : "no") << ")\n";
        std::cout << "Angular momentum   L = " << L
                   << "  (is conserved: " << (L_conserved ? "yes" : "no") << ")\n";
        std::cout << '\n';
      }
      ++frame;

      window.clear(sf::Color::Black);
      drawTrails(trails, window) //questo
      drawBodies(bodies, window);
      window.display();
    }
    
  } catch (std::exception const& e) {
    std::cerr << "Caught exception: '" << e.what() << "'\n";
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "Caught unknown exception\n";
    return EXIT_FAILURE;
  }
}