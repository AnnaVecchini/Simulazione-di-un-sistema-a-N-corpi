#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

const double G{6.67e-11};
const double ϵ{10e-12};
const double dt{0.001};
double t{0.};

struct TDvec {
  double x;
  double y;
};

TDvec operator+(TDvec const& a, TDvec const& b) {
  return {a.x + b.x, a.y + b.y};
}

TDvec operator-(TDvec const& a, TDvec const& b) {
  return {a.x - b.x, a.y - b.y};
}

TDvec operator*(TDvec const& a, double s) { return {a.x * s, a.y * s}; }

TDvec operator*(double s, TDvec const& a) { return {s * a.x, s * a.y}; }

double norm(TDvec const& a) { return sqrt(a.x * a.x + a.y * a.y); }

class Body {
 private:
  double m_;

 public:
  TDvec r;
  TDvec v;
  TDvec a;

  Body(double m, TDvec r, TDvec v, TDvec a);

  double m() const { return m_; }
};

Body::Body(double m, TDvec r, TDvec v, TDvec a) : m_{m}, r{r}, v{v}, a{a} {
  if (m_ <= 0) {
    throw std::invalid_argument("La massa deve essere positiva");
  }
  if (norm(v) >= 299792458.) {
    throw std::invalid_argument(
        "La velocita deve essere minore di quella della luce");
  }
}

void computeAccelerations(std::vector<Body>& bodies) {
  int N = bodies.size();

  for (int i{0}; i < N; ++i) {
    TDvec a_i{0, 0};

    for (int j{0}; j < N; ++j) {
      if (j != i) {
        TDvec diff = bodies[i].r - bodies[j].r;

        double dist2 = diff.x * diff.x + diff.y * diff.y;

        double denom = std::pow(dist2 + ϵ * ϵ, 1.5);

        a_i = a_i - diff * (G * bodies[j].m() / denom);
      }
    }

    bodies[i].a = a_i;
  }
}

void step(std::vector<Body>& bodies) {
  int N = bodies.size();

  std::vector<TDvec> old_a(N);
  for (int i{0}; i < N; ++i) {
    old_a[i] = bodies[i].a;
  }

  for (int i{0}; i < N; ++i) {
    bodies[i].r = bodies[i].r + bodies[i].v * dt + 0.5 * bodies[i].a * dt * dt;
  }

  computeAccelerations(bodies);

  for (int i{0}; i < N; ++i) {
    bodies[i].v = bodies[i].v + 0.5 * (bodies[i].a + old_a[i]) * dt;
  }

  t += dt;
}

double computeEnergy(std::vector<Body> const& bodies) {
  int N = bodies.size();

  double K{0.};
  for (int i{0}; i < N; ++i) {
    double v_i = norm(bodies[i].v);
    K += 0.5 * bodies[i].m() * v_i * v_i;
  }

  double U{0.};
  for (int i{0}; i < N; ++i) {
    for (int j{i + 1}; j < N; ++j) {
      double dist = norm(bodies[i].r - bodies[j].r);
      U -= G * bodies[i].m() * bodies[j].m() / dist;
    }
  }

  return K + U;
}