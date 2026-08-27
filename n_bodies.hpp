#ifndef PF_N_BODIES_HPP
#define PF_N_BODIES_HPP

#include <cmath>
#include <string>
#include <vector>

namespace pf {

// ============================================================================
// Universal constants
// ============================================================================
namespace constant {
inline constexpr double c_light{299792458.};
inline constexpr double G{6.67E-11};
}  // namespace constant

// ============================================================================
// 2D algebra
// ============================================================================
struct Vec2D {
  double x{};
  double y{};

  Vec2D &operator+=(Vec2D const &b);
};

Vec2D operator+(Vec2D a, Vec2D const &b);
Vec2D operator-(Vec2D const &a, Vec2D const &b);
Vec2D operator*(Vec2D const &a, double h);
inline Vec2D operator*(double h, Vec2D const &a) { return a * h; }
bool operator==(Vec2D const &a, Vec2D const &b);

double norm2(Vec2D const &a);
inline double norm(Vec2D const &a) { return sqrt(norm2(a)); }

// ============================================================================
// Single entity: Body
// ============================================================================
class Body {
  double m_;
  Vec2D r_;
  Vec2D v_;
  Vec2D a_;

  void validate();

 public:
  Body(double mass, Vec2D pos, Vec2D vel);
  Body(double mass, double rx, double ry, double vx, double vy);

  double get_m() const { return m_; }
  const Vec2D &get_r() const { return r_; }
  const Vec2D &get_v() const { return v_; }
  const Vec2D &get_a() const { return a_; }

  void set_a(Vec2D const &new_a) { a_ = new_a; }
  void update_position(double dt);
  void update_velocity(Vec2D const &old_a, double dt);
  void increment_a(Vec2D const &add_a) { a_ += add_a; }
};

double calculate_kinetic_energy(Body const &b);
Vec2D calculate_momentum(Body const &b);
double calculate_angular_momentum(Body const &b);

// ============================================================================
// Conserved quantities
// ============================================================================
struct SystemTotals {
  double E;
  Vec2D P;
  double L;
};

double total_kinetic_energy(std::vector<Body> const &bodies);
double total_potential_energy(std::vector<Body> const &bodies);
double total_energy(std::vector<Body> const &bodies);
Vec2D total_momentum(std::vector<Body> const &bodies);
double total_angular_momentum(std::vector<Body> const &bodies);

SystemTotals compute_totals(std::vector<Body> const &bodies);

bool is_conserved(double i_value, double f_value, double tol = 0.01);
bool is_conserved(Vec2D const &i_vec, Vec2D const &f_vec, double tol = 0.01);

// ============================================================================
// Simulation motor
// ============================================================================
std::vector<Body> load_bodies_from_file(std::string const &file);

class Simulation {
  std::vector<Body> bodies_;
  static constexpr double dt_{0.001};
  static constexpr double eps_{1E-12};

  void validate();
  void calculate_acceleration();

 public:
  Simulation(std::vector<Body> v);

  const std::vector<Body> &get_bodies() const { return bodies_; }

  void step();
};

// ============================================================================
// On screen informations
// ============================================================================
void print_total(std::string const &name, double i_value, double c_value,
                 int step);

void print_total(std::string const &name, Vec2D const &i_vec,
                 Vec2D const &c_vec, int step);

}  // namespace pf
#endif