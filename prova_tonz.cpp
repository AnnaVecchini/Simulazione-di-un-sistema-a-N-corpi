#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <vector>

#include <cstdio>
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

namespace pf {

namespace constant {
inline constexpr double c_light{300000.};
inline constexpr double G{6.67E-11};
} // namespace constant

struct Vec2D {
  double x{};
  double y{};

  Vec2D &operator+=(Vec2D const &b) {
    x += b.x;
    y += b.y;
    return *this;
  }
};

Vec2D operator+(Vec2D a, Vec2D const &b) {
  a += b;
  return a;
}

Vec2D operator-(Vec2D const &a, Vec2D const &b) {
  return {a.x - b.x, a.y - b.y};
}

Vec2D operator*(Vec2D const &a, double h) { return {a.x * h, a.y * h}; }

Vec2D operator*(double h, Vec2D const &a) { return {a.x * h, a.y * h}; }

bool operator==(Vec2D const &a, Vec2D const &b) {
  return a.x == b.x && a.y == b.y;
}

double norm2(Vec2D const &a) { return a.x * a.x + a.y * a.y; }

double norm(Vec2D const &a) { return sqrt(norm2(a)); }

class Body {
  double m_;
  Vec2D r_;
  Vec2D v_;
  Vec2D a_;

  void validate() {
    if (m_ <= 0.) {
      throw std::runtime_error{"mass must be strictly positive"};
    }
    if (norm(v_) > constant::c_light) {
      throw std::runtime_error{"can not exceed the speed of light"};
    }
  }

public:
  Body(double mass, Vec2D pos, Vec2D vel) : m_{mass}, r_{pos}, v_{vel} {
    validate();
  }

  Body(double mass, double rx, double ry, double vx, double vy)
      : Body(mass, Vec2D{rx, ry}, Vec2D{vx, vy}) {}

  double get_m() const { return m_; }
  const Vec2D &get_r() const { return r_; }
  const Vec2D &get_v() const { return v_; }
  const Vec2D &get_a() const { return a_; }

  void set_a(Vec2D const &new_a) { a_ = new_a; }

  void update_position(double dt) { r_ += v_ * dt + 0.5 * a_ * dt * dt; }

  void update_velocity(Vec2D const &old_a, double dt) {
    v_ += 0.5 * (a_ + old_a) * dt;
    validate();
  }

  void increment_a(Vec2D const &add_a) { a_ += add_a; }
};

double calculate_kinetic_energy(Body const &b) {
  return 0.5 * b.get_m() * norm2(b.get_v());
}

Vec2D calculate_momentum(Body const &b) { return b.get_m() * b.get_v(); }

double calculate_angular_momentum(Body const &b) {
  Vec2D r = b.get_r();
  Vec2D v = b.get_v();
  return b.get_m() * (r.x * v.y - r.y * v.x);
}

std::vector<Body> load_bodies_from_file(std::string const &file) {
  std::ifstream input_file(file);
  if (!input_file.is_open()) {
    throw std::runtime_error("could not open the file");
  }
  std::vector<Body> bodies;
  double mass{}, rx{}, ry{}, vx{}, vy{};
  while (input_file >> mass >> rx >> ry >> vx >> vy) {
    bodies.push_back(Body{mass, rx, ry, vx, vy});
  }
  return bodies;
}

class Simulation {
  std::vector<Body> bodies_;
  int n_steps_;
  static constexpr double dt_{0.001};
  static constexpr double eps_{1E-12};

  void validate() {
    if (bodies_.size() < 2) {
      throw std::runtime_error{"not enough bodies to run a simulation"};
    }
    if (n_steps_ < 0) {
      throw std::runtime_error{"steps must be positive"};
    }
  }

  void calculate_acceleration() {
    std::size_t N = bodies_.size();
    double eps2 = eps_ * eps_;
    std::for_each(bodies_.begin(), bodies_.end(),
                  [](Body &b) { b.set_a({0., 0.}); });
    for (std::size_t i{}; i < N - 1; ++i) {
      auto &bi = bodies_[i];
      for (std::size_t j{i + 1}; j < N; ++j) {
        auto &bj = bodies_[j];
        Vec2D diff = bj.get_r() - bi.get_r();
        double k = norm2(diff) + eps2;
        double factor = constant::G / (k * sqrt(k));
        bi.increment_a(factor * bj.get_m() * diff);
        bj.increment_a(-1. * factor * bi.get_m() * diff);
      }
    }
  }

  void step() {
    std::size_t N = bodies_.size();
    std::vector<Vec2D> old_accs(N);
    for (std::size_t i{}; i < N; ++i) {
      bodies_[i].update_position(dt_);
      old_accs[i] = bodies_[i].get_a();
    }
    calculate_acceleration();
    for (std::size_t i{}; i < N; ++i) {
      bodies_[i].update_velocity(old_accs[i], dt_);
    }
  }

public:
  Simulation(std::vector<Body> v, int n) : bodies_{std::move(v)}, n_steps_{n} {
    validate();
    calculate_acceleration();
  }

  const std::vector<Body> &get_bodies() const { return bodies_; }

  void run() {
    for (int i{}; i < n_steps_; ++i) {
      step();
    }
  }
};

double total_kinetic_energy(std::vector<Body> const &bodies) {
  return std::accumulate(bodies.begin(), bodies.end(), 0.,
                         [](double acc, Body const &b) {
                           return acc + calculate_kinetic_energy(b);
                         });
}

double total_potential_energy(std::vector<Body> const &bodies) {
  double U{};
  std::size_t N = bodies.size();
  for (std::size_t i{}; i < N - 1; ++i) {
    auto const &bi = bodies[i];
    for (std::size_t j{i + 1}; j < N; ++j) {
      auto const &bj = bodies[j];
      U -=
          constant::G * bi.get_m() * bj.get_m() / norm(bi.get_r() - bj.get_r());
    }
  }
  return U;
}

double total_energy(std::vector<Body> const &bodies) {
  return total_kinetic_energy(bodies) + total_potential_energy(bodies);
}

Vec2D total_momentum(std::vector<Body> const &bodies) {
  return std::accumulate(
      bodies.begin(), bodies.end(), Vec2D{0., 0.},
      [](Vec2D acc, Body const &b) { return acc + calculate_momentum(b); });
}

double total_angular_momentum(std::vector<Body> const &bodies) {
  return std::accumulate(bodies.begin(), bodies.end(), 0.,
                         [](double acc, Body const &b) {
                           return acc + calculate_angular_momentum(b);
                         });
}

bool is_conserved(double i_value, double f_value, double tol = 0.01) {
  double abs_diff = std::abs(f_value - i_value);
  double abs_i = std::abs(i_value);
  if (abs_i < 1E-12) {
    return abs_diff < tol;
  }
  return abs_diff < abs_i * tol;
}

bool is_conserved(Vec2D const &i_vec, Vec2D const &f_vec, double tol = 0.01) {
  double norm_diff = norm(f_vec - i_vec);
  double norm_i = norm(i_vec);
  if (norm_i < 1E-12) {
    return norm_diff < tol;
  }
  return norm_diff < tol * norm_i;
}
} // namespace pf

TEST_CASE("Testing a valid contruction of a Body") {
  pf::Body b{
      1.,
      pf::Vec2D{1., -1.},
      pf::Vec2D{2., 0.},
  };
  CHECK(b.get_m() == 1.);
  CHECK(b.get_r() == pf::Vec2D{1., -1.});
  CHECK(b.get_v() == pf::Vec2D{2., 0.});
  CHECK(b.get_a() == pf::Vec2D{0., 0.});
}

TEST_CASE("Body with negative mass throws an exception") {
  CHECK_THROWS_AS((pf::Body{
                      -1.,
                      pf::Vec2D{0., 0.},
                      pf::Vec2D{0., 0.},
                  }),
                  std::runtime_error);
}

TEST_CASE("Body with superluminar speed throws an exception") {
  CHECK_THROWS_AS((pf::Body{
                      1.,
                      pf::Vec2D{0., 0.},
                      pf::Vec2D{3e8, 0.},
                  }),
                  std::runtime_error);
}

TEST_CASE("Two equals body have same acceleration") {
  std::vector<pf::Body> bodies{
      pf::Body{1., pf::Vec2D{0., 0.}, pf::Vec2D{0., 0.}},
      pf::Body{1., pf::Vec2D{1., 0.}, pf::Vec2D{0., 0.}}};

  pf::Simulation sim(std::move(bodies), 1);

  // Il corpo 0 e' attratto verso il corpo 1 (accelerazione positiva su x)
  CHECK(sim.get_bodies()[0].get_a().x > 0.);
  // Per il terzo principio della dinamica le accelerazioni sono opposte
  // (a meno del rapporto delle masse, qui uguali)
  CHECK(sim.get_bodies()[0].get_a().x ==
        doctest::Approx(-sim.get_bodies()[1].get_a().x));
}

TEST_CASE("is_conserved correctly detects within and beyond tolerance") {
  CHECK(pf::is_conserved(100., 100.5) == true);
  CHECK(pf::is_conserved(100., 120.) == false);
  CHECK(pf::is_conserved(0., 1E-3) == true);
  CHECK(pf::is_conserved(0., 0.1) == false);
  CHECK(pf::is_conserved(pf::Vec2D{100., 150.}, pf::Vec2D{100.5, 150.5}) ==
        true);
  CHECK(pf::is_conserved(pf::Vec2D{100., 150.}, pf::Vec2D{105., 155.}) ==
        false);
  CHECK(pf::is_conserved(pf::Vec2D{0., 0.}, pf::Vec2D{1E-3, 1E-3}) == true);
  CHECK(pf::is_conserved(pf::Vec2D{0., 0.}, pf::Vec2D{0.5, 0.5}) == false);
}

TEST_CASE("load_bodies_from_file reads correctly a valid file") {
  std::string const filename{"test_input_tmp.txt"};
  {
    std::ofstream out{filename};
    out << "1. 0. 0. 0. 0.\n";
    out << "2. 1. 0. 0. 1.\n";
  }

  auto bodies = pf::load_bodies_from_file(filename);

  REQUIRE(bodies.size() == 2);
  CHECK(bodies[0].get_m() == doctest::Approx(1.));
  CHECK(bodies[1].get_m() == doctest::Approx(2.));
  CHECK(bodies[1].get_r().x == doctest::Approx(1.));
  CHECK(bodies[1].get_v().y == doctest::Approx(1.));

  std::remove(filename.c_str());
}

TEST_CASE("load_bodies_from_file throws an exception if file doesn't exists") {
  CHECK_THROWS_AS(pf::load_bodies_from_file("file_che_non_esiste.txt"),
                  std::runtime_error);
}

TEST_CASE("total energy is the sum of kinetic and potential energy") {
  std::vector<pf::Body> bodies{
      pf::Body{1., pf::Vec2D{0., 0.}, pf::Vec2D{1., 0.}},
      pf::Body{1., pf::Vec2D{1., 0.}, pf::Vec2D{0., 0.}}};
  double const E{pf::total_energy(bodies)};

  CHECK(E == doctest::Approx(pf::total_kinetic_energy(bodies) +
                             pf::total_potential_energy(bodies)));
}

TEST_CASE("total_momentum computes correctly the total momentum") {
  std::vector<pf::Body> bodies{
      pf::Body{2., pf::Vec2D{0., 0.}, pf::Vec2D{1., 0.}},
      pf::Body{3., pf::Vec2D{0., 0.}, pf::Vec2D{0., 2.}}};

  pf::Vec2D P = pf::total_momentum(bodies);
  CHECK(P == pf::Vec2D{2., 6.});
}

TEST_CASE("total_angular_momentum computes correctly the angular momentum") {
  std::vector<pf::Body> bodies{
      pf::Body{1., pf::Vec2D{1., 0.}, pf::Vec2D{0., 1.}}};

  // L = m*(x*vy - y*vx) = 1*(1*1 - 0*0) = 1
  CHECK(pf::total_angular_momentum(bodies) == doctest::Approx(1.));
}

TEST_CASE("Energy, Momentum and angular momentum are conserved during the "
          "simulation") {
  std::vector<pf::Body> bodies{
      pf::Body{1., pf::Vec2D{-0.97000436, 0.24308753},
               pf::Vec2D{0.4662036850, 0.4323657300}},
      pf::Body{2., pf::Vec2D{0.97000436, -0.24308753},
               pf::Vec2D{0.4662036850, 0.4323657300}},
      pf::Body{3., pf::Vec2D{0., 0.}, pf::Vec2D{-0.93240737, -0.86473146}}};

  double const E0 = pf::total_energy(bodies);
  pf::Vec2D const P0 = pf::total_momentum(bodies);
  double const L0 = pf::total_angular_momentum(bodies);

  pf::Simulation sim(std::move(bodies), 100);
  sim.run();
  double const E = pf::total_energy(sim.get_bodies());
  pf::Vec2D const P = pf::total_momentum(sim.get_bodies());
  double const L = pf::total_angular_momentum(sim.get_bodies());

  CHECK(pf::is_conserved(E0, E) == true);
  CHECK(pf::is_conserved(P0, P) == true);
  CHECK(pf::is_conserved(L0, L) == true);
}

/*
MAIN (just to try):

int main() {
    try {
        // 1. Carica i corpi da file (assicurati che input.txt esista)
        auto bodies = load_bodies_from_file("initial_conditions.txt");

        // Memorizziamo lo stato iniziale per verificare le conservazioni
        double E_init = mechanical_energy(bodies);
        Vec2D P_init = total_momentum(bodies);

        // 2. Istanzia ed esegui la simulazione (es. 1000 passi)
        Simulation sim(std::move(bodies), 1000);
        sim.run();

        // 3. Verifica lo stato finale
        double E_final = mechanical_energy(sim.get_bodies());
        Vec2D P_final = total_momentum(sim.get_bodies());

        std::cout << "Energia conservata: " << std::boolalpha
                  << is_conserved(E_init, E_final) << '\n';
        std::cout << "Impulso conservato: " << std::boolalpha
                  << is_conserved(P_init, P_final) << '\n';

    } catch (std::exception const& e) {
        std::cerr << "Errore: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
*/

/*
OLD FUNCTIONS (previews implementations):

double total_kinetic_energy(std::vector<Body> const& bodies) {
    double K{};
    for (auto const& b : bodies) {
        K += calculate_kinetic_energy(b);
    }
    return K;
}

double total_potential_energy(std::vector<Body> const& bodies) {
    double U{};
    std::size_t N = bodies.size();
    for (std::size_t i{}; i<N-1; ++i) {
        auto const& bi = bodies[i];
        for (std::size_t j{i+1}; j<N; ++j) {
            auto const& bj= bodies[j];
            U -= constant::G*bi.get_m()*bj.get_m()/norm(bi.get_r()-bj.get_r());
        }
    }
    return U;
}

double mechanical_energy(std::vector<Body> const& bodies) {
    return total_kinetic_energy(bodies) + total_potential_energy(bodies);
}

Vec2D total_momentum(std::vector<Body> const& bodies) {
    Vec2D P{0.,0.};
    for (auto const& b : bodies) {
        P += calculate_momentum(b);
    }
    return P;
}

double total_angular_momentum(std::vector<Body> const& bodies) {
    double L{};
    for (auto const& b : bodies) {
        L += calculate_angular_momentum(b);
    }
    return L;
}

FUNCTIONS WITH ALGORITHMS (difficult to read/longer code just look cooler... is
it really worth it?):

double total_potential_energy(std::vector<Body> const& bodies) {
    double U{};
    for (auto it=bodies.begin(); it<bodies.end(); ++it) {
        U += std::accumulate(it+1, bodies.end(),0.,
                                [&](double acc){
                                    auto next_it=it+1;
                                    return acc +
constant::G*(it->get_m())*(next_it->get_m())/norm(it->get_r()-next_it->get_r());});
    }
    return U;
}

double total_potential_energy(std::vector<Body> const& bodies) {
    auto i = bodies.begin();
    return std::accumulate(i, bodies.end()-1, 0.,
                        [&](double acc_i){
                            auto j = i+1;
                            return acc_i + std::accumulate(j, bodies.end(), 0.,
                                                [&](double acc_j){
                                                    return acc_j +
constant::G*(i->get_m())*(j->get_m())/norm(i->get_r()-j->get_r());});
                                                });
}

void step() {
        std::size_t N = bodies_.size();
        std::vector<Vec2D> old_accs(N);
        std::for_each(bodies_.begin(), bodies_.end(), [](Body
&b){b.update_position(dt_);}); std::transform(bodies_.begin(), bodies_.end(),
old_accs.begin(), [](Body &b){return b.get_a();}); calculate_acceleration(); for
(std::size_t i{}; i<N; ++i) { bodies_[i].update_velocity(old_accs[i],dt_);
        }

    }

RIP (use of algorithm preferred ;)):

for (auto &b : bodies_) {
    b.set_a({0.,0.});
}

WHY WOULD WE NEED THIS? :

TEST_CASE("step conserves energy approximately over a few steps") {
  double const m{1. / pf::G};
  std::vector<pf::Body> bodies{
      pf::Body{m, pf::TDvec{-0.97000436, 0.24308753},
               pf::TDvec{0.4662036850, 0.4323657300}, pf::TDvec{0., 0.}},
      pf::Body{m, pf::TDvec{0.97000436, -0.24308753},
               pf::TDvec{0.4662036850, 0.4323657300}, pf::TDvec{0., 0.}},
      pf::Body{m, pf::TDvec{0., 0.}, pf::TDvec{-0.93240737, -0.86473146},
               pf::TDvec{0., 0.}}};

  pf::computeAccelerations(bodies);
  double const E0{pf::computeEnergy(bodies)};

  for (int i{0}; i < 100; ++i) {
    pf::step(bodies);
  }

  double const E{pf::computeEnergy(bodies)};
  CHECK(pf::isEnergyConserved(E0, E, 0.01) == true);
}
*/