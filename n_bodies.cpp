#include "n_bodies.hpp"

#include <fstream>
#include <iostream>
#include <numeric>
#include <stdexcept>

namespace pf {

// ============================================================================
// 2D algebra
// ============================================================================
Vec2D &Vec2D::operator+=(Vec2D const &b) {
  x += b.x;
  y += b.y;
  return *this;
}

Vec2D operator+(Vec2D a, Vec2D const &b) {
  a += b;
  return a;
}

Vec2D operator-(Vec2D const &a, Vec2D const &b) {
  return {a.x - b.x, a.y - b.y};
}

Vec2D operator*(Vec2D const &a, double h) { return {a.x * h, a.y * h}; }

bool operator==(Vec2D const &a, Vec2D const &b) {
  return a.x == b.x && a.y == b.y;
}

double norm2(Vec2D const &a) { return a.x * a.x + a.y * a.y; }

// ============================================================================
// Single entity: Body
// ============================================================================
void Body::validate() {
  if (m_ <= 0.) {
    throw std::runtime_error{"mass must be strictly positive"};
  }
  if (norm(v_) > constant::c_light) {
    throw std::runtime_error{"can not exceed the speed of light"};
  }
}

Body::Body(double mass, Vec2D pos, Vec2D vel) : m_{mass}, r_{pos}, v_{vel} {
  validate();
}

Body::Body(double mass, double rx, double ry, double vx, double vy)
    : Body(mass, Vec2D{rx, ry}, Vec2D{vx, vy}) {}

void Body::update_position(double dt) { r_ += v_ * dt + 0.5 * a_ * dt * dt; }

void Body::update_velocity(Vec2D const &old_a, double dt) {
  v_ += 0.5 * (a_ + old_a) * dt;
  validate();
}

double calculate_kinetic_energy(Body const &b) {
  return 0.5 * b.get_m() * norm2(b.get_v());
}

Vec2D calculate_momentum(Body const &b) { return b.get_m() * b.get_v(); }

double calculate_angular_momentum(Body const &b) {
  Vec2D r = b.get_r();
  Vec2D v = b.get_v();
  return b.get_m() * (r.x * v.y - r.y * v.x);
}

// ============================================================================
// Conserved quantities
// ============================================================================
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

SystemTotals compute_totals(std::vector<Body> const &bodies) {
  return {total_energy(bodies), total_momentum(bodies),
          total_angular_momentum(bodies)};
}

bool is_conserved(double i_value, double f_value, double tol) {
  double abs_diff = std::abs(f_value - i_value);
  double abs_i = std::abs(i_value);
  if (abs_i < 1E-12) {
    return abs_diff < tol;
  }
  return abs_diff < abs_i * tol;
}

bool is_conserved(Vec2D const &i_vec, Vec2D const &f_vec, double tol) {
  double norm_diff = norm(f_vec - i_vec);
  double norm_i = norm(i_vec);
  if (norm_i < 1E-12) {
    return norm_diff < tol;
  }
  return norm_diff < tol * norm_i;
}

// ============================================================================
// Simulation motor
// ============================================================================
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

void Simulation::validate() {
  if (bodies_.size() < 2) {
    throw std::runtime_error{"not enough bodies to run a simulation"};
  }
}

void Simulation::calculate_acceleration() {
  std::size_t N = bodies_.size();
  double eps2 = eps_ * eps_;
  for (auto &b : bodies_) {
    b.set_a({0., 0.});
  }
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

Simulation::Simulation(std::vector<Body> v) : bodies_{std::move(v)} {
  validate();
  calculate_acceleration();
}

void Simulation::step() {
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

// ============================================================================
// On screen informations
// ============================================================================
void print_if_not_conserved(char name, double i_value, double c_value,
                            int step) {
  if (!is_conserved(i_value, c_value)) {
    std::cout << "CONSERVATION FAILED at step: " << step << '\n'
              << name << " = " << c_value << " (\033[31mnot conserved\033[0m)"
              << "\n\n";  // prints "not conserved" red
  }
}

void print_if_not_conserved(char name, Vec2D const &i_vec, Vec2D const &c_vec,
                            int step) {
  if (!is_conserved(i_vec, c_vec)) {
    std::cout << "CONSERVATION FAILED at step: " << step << '\n'
              << name << " = (" << c_vec.x << ", " << c_vec.y
              << ") (\033[31mnot conserved\033[0m)"
              << "\n\n";  // prints "not conserved" red
  }
}
}  // namespace pf