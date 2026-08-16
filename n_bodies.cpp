#include "n_bodies.hpp"

#include <cstddef>
#include <fstream>

namespace pf {

std::vector<Body> readBodiesFromFile(std::string const& filename) {
  std::ifstream input{filename};
  if (!input) {
    throw std::runtime_error("Impossibile aprire il file: " + filename);
  }

  std::vector<Body> bodies;
  double m{};
  double x{};
  double y{};
  double vx{};
  double vy{};
  while (input >> m >> x >> y >> vx >> vy) {
    bodies.push_back(Body{m, TDvec{x, y}, TDvec{vx, vy}, TDvec{0., 0.}});
  }

  return bodies;
}

Body::Body(double mass, TDvec pos, TDvec vel, TDvec acc)
    : m_{mass}, r{pos}, v{vel}, a{acc} {
  if (m_ <= 0) {
    throw std::invalid_argument("La massa deve essere positiva");
  }
  if (norm(v) >= c_light) {
    throw std::invalid_argument(
        "La velocita deve essere minore di quella della luce");
  }
}

void computeAccelerations(std::vector<Body>& bodies) {
  std::size_t N = bodies.size();

  for (std::size_t i{0}; i < N; ++i) {
    TDvec a_i{0, 0};

    for (std::size_t j{0}; j < N; ++j) {
      if (j != i) {
        TDvec diff = bodies[i].r - bodies[j].r;

        double dist2 = diff.x * diff.x + diff.y * diff.y;

        double denom = std::pow(dist2 + eps * eps, 1.5);

        a_i = a_i - diff * (G * bodies[j].m() / denom);
      }
    }

    bodies[i].a = a_i;
  }
}

void step(std::vector<Body>& bodies) {
  std::size_t N = bodies.size();

  // Salvo le vecchie accelerazioni a(t), prima di aggiornare le posizioni
  std::vector<TDvec> old_a(N);
  for (std::size_t i{0}; i < N; ++i) {
    old_a[i] = bodies[i].a;
  }

  // Primo ciclo: aggiorno TUTTE le posizioni usando v(t) e a(t) correnti
  for (std::size_t i{0}; i < N; ++i) {
    bodies[i].r = bodies[i].r + bodies[i].v * dt + 0.5 * bodies[i].a * dt * dt;
  }

  // Ricalcolo le accelerazioni a(t+dt) alle nuove posizioni
  computeAccelerations(bodies);

  // Secondo ciclo: aggiorno TUTTE le velocita' usando la media
  // tra a(t) (old_a) e a(t+dt) (bodies[i].a, gia' ricalcolata)
  for (std::size_t i{0}; i < N; ++i) {
    bodies[i].v = bodies[i].v + 0.5 * (bodies[i].a + old_a[i]) * dt;
  }

  t += dt;
}

double computeEnergy(std::vector<Body> const& bodies) {
  std::size_t N = bodies.size();

  // Energia cinetica: somma di 1/2 * m * v^2 per ogni corpo
  double K{0.};
  for (std::size_t i{0}; i < N; ++i) {
    double v_i = norm(bodies[i].v);
    K += 0.5 * bodies[i].m() * v_i * v_i;
  }

  // Energia potenziale: somma sulle coppie i<j, per non contarle due volte
  double U{0.};
  for (std::size_t i{0}; i < N; ++i) {
    for (std::size_t j{i + 1}; j < N; ++j) {
      double dist = norm(bodies[i].r - bodies[j].r);
      U -= G * bodies[i].m() * bodies[j].m() / dist;
    }
  }

  return K + U;
}

bool isEnergyConserved(double E0, double E, double tolerance) {
  return std::abs(E - E0) <= tolerance * std::abs(E0);
}

TDvec computeMomentum(std::vector<Body> const& bodies) {
  std::size_t N = bodies.size();

  TDvec P{0., 0.};
  for (std::size_t i{0}; i < N; ++i) {
    P = P + bodies[i].m() * bodies[i].v;
  }

  return P;
}

double computeAngularMomentum(std::vector<Body> const& bodies) {
  std::size_t N = bodies.size();

  double L{0.};
  for (std::size_t i{0}; i < N; ++i) {
    L += bodies[i].m() *
         (bodies[i].r.x * bodies[i].v.y - bodies[i].r.y * bodies[i].v.x);
  }

  return L;
}

bool isMomentumConserved(std::vector<Body> const& bodies, TDvec const& P0,
                          double tolerance) {
  std::size_t N = bodies.size();

  // Scala tipica della quantita' di moto dei singoli corpi, usata
  // come riferimento al posto di P0 (che puo' essere nullo)
  double scale{0.};
  for (std::size_t i{0}; i < N; ++i) {
    scale += bodies[i].m() * norm(bodies[i].v);
  }

  TDvec P{computeMomentum(bodies)};
  return norm(P - P0) <= tolerance * scale;
}

bool isAngularMomentumConserved(std::vector<Body> const& bodies, double L0,
                                 double tolerance) {
  std::size_t N = bodies.size();

  // Scala tipica del momento angolare dei singoli corpi
  double scale{0.};
  for (std::size_t i{0}; i < N; ++i) {
    scale += bodies[i].m() * norm(bodies[i].r) * norm(bodies[i].v);
  }

  double L{computeAngularMomentum(bodies)};
  return std::abs(L - L0) <= tolerance * scale;
}

}  // namespace pf
