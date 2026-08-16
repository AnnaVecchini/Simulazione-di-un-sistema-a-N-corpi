#include <cstdlib>
#include <exception>
#include <iostream>
#include <vector>

#include "n_bodies.hpp"

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

    int const n_steps{10000};
    for (int i{0}; i < n_steps; ++i) {
      pf::step(bodies);

      if (i % 1000 == 0) {
        double E{pf::computeEnergy(bodies)};
        bool E_conserved{pf::isEnergyConserved(E0, E, tolerance)};
        bool P_conserved{pf::isMomentumConserved(bodies, P0, tolerance)};
        bool L_conserved{pf::isAngularMomentumConserved(bodies, L0, tolerance)};

        std::cout << "t = " << pf::t << "  E = " << E
                  << "  E is conserved: " << (E_conserved ? "yes" : "no")
                  << "  P is conserved: " << (P_conserved ? "yes" : "no")
                  << "  L is conserved: " << (L_conserved ? "yes" : "no") << '\n';
      }
    }

  } catch (std::exception const& e) {
    std::cerr << "Caught exception: '" << e.what() << "'\n";
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "Caught unknown exception\n";
    return EXIT_FAILURE;
  }
}
