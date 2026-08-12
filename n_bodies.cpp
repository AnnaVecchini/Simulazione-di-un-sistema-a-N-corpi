#include <cmath>
#include <iostream>
#include <vector>

const double G{6.67e-11};
const double ϵ{10e-12};
const double Δt{0.001};

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

double norm(TDvec const& a) { return sqrt(a.x * a.x + a.y * a.y); }

class Body {
 private:
  double m_;
  TDvec r_;
  TDvec v_;
  TDvec a_;

 public:
  Body(double m, TDvec r, TDvec v, TDvec a = {0., 0.})
      : m_{m},
        r_{r},
        v_{v},
        a_{a}  // initialization list
  {
    if (m_ <= 0.) {
      throw std::invalid_argument{"la massa di un corpo deve essere positiva"};
    }
  }
};
