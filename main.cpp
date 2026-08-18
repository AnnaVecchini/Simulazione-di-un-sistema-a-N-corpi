#include <cstddef>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <vector>

#include <SFML/Graphics.hpp>

#include "n_bodies.hpp"

// Dimensioni della finestra, in pixel
constexpr unsigned int window_width{800};
constexpr unsigned int window_height{600};

// Pixel per unita' di posizione della simulazione (le posizioni nel
// caso Figure-8 sono dell'ordine di 1)
constexpr float scale{200.f};

// Quanti step di simulazione eseguire per ogni fotogramma disegnato:
// con dt molto piccolo (0.001), un solo step per frame renderebbe il
// moto impercettibilmente lento
constexpr int steps_per_frame{20};

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

// Disegna tutti i corpi come cerchi bianchi nella finestra
void drawBodies(std::vector<pf::Body> const& bodies, sf::RenderWindow& window) {
  float const radius{6.f};

  for (std::size_t i{0}; i < bodies.size(); ++i) {
    sf::CircleShape shape{radius};
    shape.setFillColor(sf::Color::White);
    shape.setOrigin(radius, radius);
    shape.setPosition(toScreenCoordinates(bodies[i].r));
    window.draw(shape);
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

    // Inizializzo le accelerazioni a(0) PRIMA del ciclo: step() si
    // aspetta che bodies[i].a contenga gia' l'accelerazione corrente
    pf::computeAccelerations(bodies);

    double const E0{pf::computeEnergy(bodies)};
    pf::TDvec const P0{pf::computeMomentum(bodies)};
    double const L0{pf::computeAngularMomentum(bodies)};
    double const tolerance{0.01};  // 1% di tolleranza relativa

    sf::RenderWindow window{sf::VideoMode(window_width, window_height),
                             "Simulazione N-Body"};
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

      // Ogni ~60 frame (circa una volta al secondo, con il limite di
      // 60 fps impostato sopra) stampiamo un controllo di conservazione
      if (frame % 60 == 0) {
        double const E{pf::computeEnergy(bodies)};
        bool const E_conserved{pf::isEnergyConserved(E0, E, tolerance)};
        bool const P_conserved{pf::isMomentumConserved(bodies, P0, tolerance)};
        bool const L_conserved{
            pf::isAngularMomentumConserved(bodies, L0, tolerance)};

        std::cout << "t = " << pf::t << "  E = " << E
                  << "  E conservata: " << (E_conserved ? "si" : "no")
                  << "  P conservata: " << (P_conserved ? "si" : "no")
                  << "  L conservato: " << (L_conserved ? "si" : "no") << '\n';
      }
      ++frame;

      window.clear(sf::Color::Black);
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